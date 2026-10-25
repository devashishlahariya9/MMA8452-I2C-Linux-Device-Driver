/**
 * @author Devashish Lahariya
 * @date 25-10-2026
 * @brief A simple device driver for the MMA8452 3-Axis Accelerometer.
 * 
 * @note This software is free and is provided AS IS and comes with no warranty.
 * Does it work? Yes.
 * Should you use it in production? No.
 * Do I take any responsibility for your stuff? Absolutely not.
 * Feel free to refer this as a learning guide for your own stuff :)
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

// BSC register offsets
#define BSC_C_REG 				0x00
#define BSC_S_REG 				0x04
#define BSC_DLEN_REG 			0x08
#define BSC_A_REG 				0x0C
#define BSC_FIFO_REG 			0x10
#define BSC_DIV_REG 			0x14
#define BSC_DEL_REG 			0x18
#define BSC_CLKT_REG 			0x1C

// Control register bit positions
#define BSC_C_I2CEN 			15
#define BSC_C_ST 				7
#define BSC_C_CLEAR 			4
#define BSC_C_READ 				0

// Status register bit positions
#define BSC_S_CLKT 				9
#define BSC_S_ERR 				8
#define BSC_S_RXF 				7
#define BSC_S_TXE 				6
#define BSC_S_RXD 				5
#define BSC_S_TXD 				4
#define BSC_S_TXW               2
#define BSC_S_DONE 				1
#define BSC_S_TA 				0

#define BSC_STATUS_CLEAR 		(BIT(BSC_S_DONE) | \
						  		BIT(BSC_S_ERR) |  \
						  		BIT(BSC_S_CLKT))

#define I2C_SCL_FREQ 			100000
#define I2C_TIMEOUT_US 			100000

struct i2c_peripheral
{
	void __iomem *base;

	struct device *dev;
	struct delayed_work data_fetch_work;
};

enum i2c_start_mode
{
	I2C_START_MODE_WRITE = 0,
	I2C_START_MODE_READ
};

///////////////////// General helpers /////////////////////

static u32 i2c_read_register(struct i2c_peripheral *i2c, u32 reg)
{
	return readl(i2c->base + reg);
}

static void i2c_write_register(struct i2c_peripheral *i2c, u32 reg, u32 val)
{
	writel(val, i2c->base + reg);
}

static void i2c_disable_clock(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static void i2c_disable_controller(void *data)
{
	struct i2c_peripheral *i2c = data;

	i2c_write_register(i2c, BSC_C_REG, 0);
}

static int i2c_init(struct platform_device *pdev, struct i2c_peripheral **i2c_out)
{
	struct device *dev = &pdev->dev;
	struct i2c_peripheral *i2c;
	struct clk *clk;
	unsigned long clock_rate;
	u32 clock_divider;
	u32 val;
	int ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;
	
	i2c->dev = dev;

	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "[%s]: Unable to get I2C clock\n", __func__);

	ret = clk_prepare_enable(clk);
	if (ret)
		return dev_err_probe(dev, ret, "[%s]: Unable to enable I2C clock\n", __func__);

	ret = devm_add_action_or_reset(dev, i2c_disable_clock, clk);
	if (ret)
		return ret;

	clock_rate = clk_get_rate(clk);
	if (!clock_rate)
		return -EINVAL;

	dev_info(dev, "[%s]: I2C clock rate: %lu Hz\n", __func__, clock_rate);

	clock_divider = DIV_ROUND_UP(clock_rate, I2C_SCL_FREQ);

	// BCM BSC requires an even divider.
	if (clock_divider & 1)
		clock_divider++;

	if (clock_divider < 2 || clock_divider > 0xFFFE)
		return -EINVAL;

	// Disable controller before configuring it.
	i2c_write_register(i2c, BSC_C_REG, 0);

	i2c_write_register(i2c, BSC_DIV_REG, clock_divider);

	val = i2c_read_register(i2c, BSC_DIV_REG) & 0xFFFF;
	if (val != clock_divider)
	{
		dev_err(dev, "[%s]: Divider mismatch: expected %u, got %u\n", __func__, clock_divider, val);
		return -EINVAL;
	}

	dev_info(dev, "[%s]: I2C divider: %u\n", __func__, clock_divider);

	// Enable controller and clear FIFO.
	i2c_write_register(i2c, BSC_C_REG, BIT(BSC_C_I2CEN) | (3U << BSC_C_CLEAR));

	// Register cleanup before clock cleanup so the controller is disabled while its clock is running.
	ret = devm_add_action_or_reset(dev, i2c_disable_controller, i2c);
	if (ret)
		return ret;

	*i2c_out = i2c;

	return 0;
}

static void i2c_set_slave_address(struct i2c_peripheral *i2c, u8 addr)
{
	i2c_write_register(i2c, BSC_A_REG, addr & 0x7F);
}

static void i2c_set_data_length(struct i2c_peripheral *i2c, u16 len)
{
	i2c_write_register(i2c, BSC_DLEN_REG, len);
}

static void i2c_clear_status(struct i2c_peripheral *i2c)
{
	// Write-one-to-clear status bits.
	i2c_write_register(i2c, BSC_S_REG, BSC_STATUS_CLEAR);
}

static int i2c_wait_for_done(struct i2c_peripheral *i2c)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(i2c->base + BSC_S_REG,
							 status,
							 status & (BIT(BSC_S_DONE) |
									   BIT(BSC_S_ERR) |
									   BIT(BSC_S_CLKT)),
							 10, I2C_TIMEOUT_US);

	if (ret)
		return ret;

	if (status & BIT(BSC_S_CLKT))
		return -ETIMEDOUT;

	if (status & BIT(BSC_S_ERR))
		return -ENXIO;

	return 0;
}

static int i2c_start_transmission(struct i2c_peripheral *i2c, enum i2c_start_mode mode)
{
	u32 status;
	u32 control;
	int ret;

	// Ensure the controller is idle.
	ret = readl_poll_timeout(i2c->base + BSC_S_REG,
							 status,
							 !(status & BIT(BSC_S_TA)),
							 10, I2C_TIMEOUT_US);
	if (ret)
		return ret;

	i2c_clear_status(i2c);

	control = BIT(BSC_C_I2CEN) | BIT(BSC_C_ST);

	if (mode == I2C_START_MODE_READ)
		control |= BIT(BSC_C_READ);

	i2c_write_register(i2c, BSC_C_REG, control);

	return 0;
}

static int i2c_send_data(struct i2c_peripheral *i2c, const u8 *buf, u16 len)
{
	u32 status;
	int ret;
	u16 i;

	if (!i2c || !buf || !len || len > 16)
		return -EINVAL;

	// Wait until the controller is idle.
	ret = readl_poll_timeout(i2c->base + BSC_S_REG,
				 status,
				 !(status & BIT(BSC_S_TA)),
				 10, I2C_TIMEOUT_US);
	if (ret)
		return ret;

	// Clear previous DONE, ERR, and CLKT flags.
	i2c_clear_status(i2c);

	// Clear FIFO while preserving controller enable.
	i2c_write_register(i2c, BSC_C_REG, BIT(BSC_C_I2CEN) | (3U << BSC_C_CLEAR));

	i2c_set_data_length(i2c, len);

	// Fill the TX FIFO before starting transmission.
	for (i = 0; i < len; i++)
	{
		status = i2c_read_register(i2c, BSC_S_REG);

		if (!(status & BIT(BSC_S_TXD)))
			return -EIO;

		i2c_write_register(i2c, BSC_FIFO_REG, buf[i]);
	}

	ret = i2c_start_transmission(i2c, I2C_START_MODE_WRITE);
	if (ret)
		return ret;

	return i2c_wait_for_done(i2c);
}

static int i2c_receive_data(struct i2c_peripheral *i2c, u8 *buf, u16 len)
{
	u32 status;
	int ret;
	u16 i;

	if (!buf || !len || len > 16)
		return -EINVAL;

	// Clear receive FIFO before the transaction.
	i2c_write_register(i2c, BSC_C_REG, BIT(BSC_C_I2CEN) | (3U << BSC_C_CLEAR));

	i2c_set_data_length(i2c, len);

	ret = i2c_start_transmission(i2c, I2C_START_MODE_READ);
	if (ret)
		return ret;

	for (i = 0; i < len; i++)
	{
		ret = readl_poll_timeout(i2c->base + BSC_S_REG,
								 status,
								 status & (BIT(BSC_S_RXD) |
										   BIT(BSC_S_ERR) |
										   BIT(BSC_S_CLKT)),
								 10, I2C_TIMEOUT_US);
		if (ret)
			return ret;

		if (status & BIT(BSC_S_CLKT))
			return -ETIMEDOUT;

		if (status & BIT(BSC_S_ERR))
			return -ENXIO;

		buf[i] = i2c_read_register(i2c, BSC_FIFO_REG) & 0xFF;
	}

	return i2c_wait_for_done(i2c);
}

///////////////////// MMA8452 specific /////////////////////

#define MMA8452_STATUS_REG					0x00
#define MMA8452_OUT_X_MSB_REG    			0x01
#define MMA8452_SYSMOD_REG					0x0B
#define MMA8452_WHO_AM_I_REG 				0x0D
#define MMA8452_XYZ_DATA_CFG_REG  			0x0E
#define MMA8452_CTRL_REG1					0x2A
#define MMA8452_CTRL_REG2					0x2B
#define MMA8452_CTRL_REG3					0x2C
#define MMA8452_CTRL_REG4					0x2D
#define MMA8452_CTRL_REG5					0x2E

#define MMA8452_STATUS_ZYXDR				BIT(3)
#define MMA8452_STATUS_ZYXOW				BIT(7)

#define MMA8452_AXIS_DATA_LEN    			6

#define MMA8542_I2C_ADDR 					0x1C
#define MMA8542_DEVICE_ID					0x2A

// CTRL_REG1 Bits
#define MMA8452_CTRL_REG1_ASLP_RATE1		BIT(7)
#define MMA8452_CTRL_REG1_ASLP_RATE0		BIT(6)
#define MMA8452_CTRL_REG1_DR2				BIT(5)
#define MMA8452_CTRL_REG1_DR1				BIT(4)
#define MMA8452_CTRL_REG1_DR0				BIT(3)
#define MMA8452_CTRL_REG1_LNOISE			BIT(2)
#define MMA8452_CTRL_REG1_F_READ			BIT(1)
#define MMA8452_CTRL_REG1_ACTIVE			BIT(0)

// CTRL_REG2 Bits
#define MMA8452_CTRL_REG2_ST				BIT(7)
#define MMA8452_CTRL_REG2_RST				BIT(6)
#define MMA8452_CTRL_REG2_SMODS1			BIT(4)
#define MMA8452_CTRL_REG2_SMODS0			BIT(3)
#define MMA8452_CTRL_REG2_SLPE				BIT(2)
#define MMA8452_CTRL_REG2_MODS1				BIT(1)
#define MMA8452_CTRL_REG2_MODS0				BIT(0)

#define MMA8452_FETCH_INTERVAL_MS 			500

typedef enum
{
	MMA8452_SYS_MODE_STANDBY = 0,
	MMA8452_SYS_MODE_WAKE,
	MMA8452_SYS_MODE_SLEEP,
} eMMA8452_SYS_MODE;

typedef enum
{
	MMA8452_ASLP_RATE_50_Hz = 0,
	MMA8452_ASLP_RATE_12_5_Hz,				// 12.5 Hz
	MMA8452_ASLP_RATE_6_25_Hz,				// 6.25 Hz
	MMA8452_ASLP_RATE_1_56_Hz,				// 1.56 Hz
} eMMA8452_ASLP_RATE;

typedef enum
{
	MMA8452_OUT_DATA_RATE_800_Hz = 0,
	MMA8452_OUT_DATA_RATE_400_Hz,
	MMA8452_OUT_DATA_RATE_200_Hz,
	MMA8452_OUT_DATA_RATE_100_Hz,
	MMA8452_OUT_DATA_RATE_50_Hz,
	MMA8452_OUT_DATA_RATE_12_5_Hz,			// 12.5 Hz
	MMA8452_OUT_DATA_RATE_6_25_Hz,			// 6.25 Hz
	MMA8452_OUT_DATA_RATE_1_56_Hz,			// 1.56 Hz
} eMMA8452_OUT_DATA_RATE;

typedef enum
{
	MMA8452_NOISE_MODE_NORMAL = 0,
	MMA8452_NOISE_MODE_REDUCED,
} eMMA8452_NOISE_MODE;

typedef enum
{
	MMA8452_OP_MODE_STANDBY = 0,
	MMA8452_OP_MODE_ACTIVE,
} eMMA8452_OP_MODE;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} MMA8452_t;

static int mma8452_read_register(struct device *dev, struct i2c_peripheral *i2c, u8 reg, u8 *value)
{
	u32 status;
	int ret;

	if (!value)
		return -EINVAL;

	ret = readl_poll_timeout(i2c->base + BSC_S_REG,
							 status,
							 !(status & BIT(BSC_S_TA)),
							 10, I2C_TIMEOUT_US);
	if (ret)
		return ret;

	i2c_clear_status(i2c);

	i2c_write_register(i2c, BSC_C_REG,
					   BIT(BSC_C_I2CEN) | (3U << BSC_C_CLEAR));

	i2c_set_data_length(i2c, 1);

	i2c_write_register(i2c, BSC_C_REG,
					   BIT(BSC_C_I2CEN) | BIT(BSC_C_ST));

	ret = readl_poll_timeout_atomic(i2c->base + BSC_S_REG,
									status,
									status & (BIT(BSC_S_TXW) |
											  BIT(BSC_S_ERR) |
											  BIT(BSC_S_CLKT) |
											  BIT(BSC_S_DONE)),
									0, I2C_TIMEOUT_US);
	if (ret)
		goto reset;

	if (status & BIT(BSC_S_CLKT))
	{
		ret = -ETIMEDOUT;
		goto reset;
	}

	if (status & BIT(BSC_S_ERR))
	{
		ret = -ENXIO;
		goto reset;
	}

	if (!(status & BIT(BSC_S_TXW)))
	{
		ret = -EIO;
		goto reset;
	}

	i2c_write_register(i2c, BSC_FIFO_REG, reg);

	i2c_set_data_length(i2c, 1);

	i2c_write_register(i2c, BSC_C_REG,
					   		BIT(BSC_C_I2CEN) |
						   	BIT(BSC_C_ST) |
						   	BIT(BSC_C_READ));

	ret = readl_poll_timeout_atomic(i2c->base + BSC_S_REG,
									status,
									status & (BIT(BSC_S_DONE) |
											  BIT(BSC_S_ERR) |
											  BIT(BSC_S_CLKT)),
									0, I2C_TIMEOUT_US);
	if (ret)
		goto reset;

	if (status & BIT(BSC_S_CLKT))
	{
		ret = -ETIMEDOUT;
		goto reset;
	}

	if (status & BIT(BSC_S_ERR))
	{
		ret = -ENXIO;
		goto reset;
	}

	if (!(status & BIT(BSC_S_RXD)))
	{
		ret = -EIO;
		goto reset;
	}

	*value = i2c_read_register(i2c, BSC_FIFO_REG) & 0xFF;

	i2c_clear_status(i2c);

	return 0;

reset:
	dev_err(dev,
			"[%s]: Transfer failed: %d, BSC_S=0x%08x, BSC_C=0x%08x\n",
			__func__, ret,
			i2c_read_register(i2c, BSC_S_REG),
			i2c_read_register(i2c, BSC_C_REG));

	i2c_write_register(i2c, BSC_C_REG, 0);
	i2c_clear_status(i2c);
	i2c_write_register(i2c, BSC_C_REG,
					   BIT(BSC_C_I2CEN) |
						   (3U << BSC_C_CLEAR));

	return ret;
}

static int mma8452_write_register(struct device *dev, struct i2c_peripheral *i2c, u8 reg, u8 value)
{
    u8 buf[2] = { reg, value };
    int ret;

    if (!dev || !i2c)
        return -EINVAL;

    ret = i2c_send_data(i2c, buf, sizeof(buf));
    if (ret)
    {
        dev_err(dev, "[%s]: Failed to write 0x%02x to register 0x%02x: %d\n", __func__, value, reg, ret);
        return ret;
    }

    return 0;
}

static int mma8452_get_system_mode(struct device *dev, struct i2c_peripheral *i2c, eMMA8452_SYS_MODE *mode)
{
	u8 reg_value;
	u8 sys_mode;

	const char *sys_mode_str[] = {
		[MMA8452_SYS_MODE_STANDBY] = "Standby Mode",
		[MMA8452_SYS_MODE_WAKE]    = "Wake Mode",
		[MMA8452_SYS_MODE_SLEEP]   = "Sleep Mode",
	};

	int ret;

	if (!mode)
		return -EINVAL;

	ret = mma8452_read_register(dev, i2c, MMA8452_SYSMOD_REG, &reg_value);
	if (ret)
		return ret;

	sys_mode = reg_value & 0x03;

	if (sys_mode > MMA8452_SYS_MODE_SLEEP)
	{
		dev_err(dev, "[%s]: Invalid system mode: %u\n", __func__, sys_mode);
		return -EINVAL;
	}

	*mode = (eMMA8452_SYS_MODE)sys_mode;

	dev_info(dev, "[%s]: Current Mode: %s\n", __func__, sys_mode_str[sys_mode]);

	return 0;
}

static int mma8452_soft_reset(struct device *dev, struct i2c_peripheral *i2c)
{
	u8 ctrl_reg2;
	int ret;

	if (!dev || !i2c)
		return -EINVAL;

	ret = mma8452_read_register(dev, i2c, MMA8452_CTRL_REG2, &ctrl_reg2);
	if (ret)
		return ret;

	ctrl_reg2 |= BIT(6);

	ret = mma8452_write_register(dev, i2c, MMA8452_CTRL_REG2, ctrl_reg2);
	if (ret)
		return ret;

	msleep(10);

	dev_info(dev, "[%s]: MMA8452 soft reset requested\n", __func__);

	return 0;
}

static int mma8452_init(struct device *dev, struct i2c_peripheral *i2c, u8 slave_addr)
{
	u8 device_id;
	u8 ctrl_reg1;
	u8 ctrl_reg2;
	u8 xyz_data_cfg;
	eMMA8452_OUT_DATA_RATE data_rate = MMA8452_OUT_DATA_RATE_100_Hz;
	eMMA8452_ASLP_RATE sleep_rate = MMA8452_ASLP_RATE_50_Hz;
	eMMA8452_NOISE_MODE noise_mode = MMA8452_NOISE_MODE_NORMAL;
	eMMA8452_OP_MODE op_mode = MMA8452_OP_MODE_ACTIVE;
	int ret;

	if (!dev || !i2c)
		return -EINVAL;

	i2c_set_slave_address(i2c, slave_addr);

	ret = mma8452_read_register(dev, i2c, MMA8452_WHO_AM_I_REG, &device_id);
	if (ret)
		return ret;

	if (device_id != MMA8542_DEVICE_ID)
	{
		dev_err(dev, "[%s]: Unexpected device ID: 0x%02x\n", __func__, device_id);
		return -ENODEV;
	}

	ret = mma8452_soft_reset(dev, i2c);
	if (ret)
		return ret;

	msleep(500);

	ret = mma8452_read_register(dev, i2c, MMA8452_CTRL_REG1, &ctrl_reg1);
	if (ret)
		return ret;

	ctrl_reg1 &= ~MMA8452_CTRL_REG1_ACTIVE;

	ret = mma8452_write_register(dev, i2c, MMA8452_CTRL_REG1, ctrl_reg1);
	if (ret)
		return ret;

	ret = mma8452_read_register(dev, i2c, MMA8452_CTRL_REG2, &ctrl_reg2);
	if (ret)
		return ret;

	ctrl_reg2 &= ~MMA8452_CTRL_REG2_SLPE;

	ret = mma8452_write_register(dev, i2c, MMA8452_CTRL_REG2, ctrl_reg2);
	if (ret)
		return ret;

	ret = mma8452_read_register(dev, i2c, MMA8452_XYZ_DATA_CFG_REG, &xyz_data_cfg);
	if (ret)
		return ret;

	xyz_data_cfg &= ~GENMASK(1, 0);

	ret = mma8452_write_register(dev, i2c, MMA8452_XYZ_DATA_CFG_REG, xyz_data_cfg);
	if (ret)
		return ret;

	ctrl_reg1 &= ~(MMA8452_CTRL_REG1_ASLP_RATE1 |
		       MMA8452_CTRL_REG1_ASLP_RATE0 |
		       MMA8452_CTRL_REG1_DR2 |
		       MMA8452_CTRL_REG1_DR1 |
		       MMA8452_CTRL_REG1_DR0 |
		       MMA8452_CTRL_REG1_LNOISE |
		       MMA8452_CTRL_REG1_F_READ |
		       MMA8452_CTRL_REG1_ACTIVE);

	ctrl_reg1 |= (sleep_rate << 6);
	ctrl_reg1 |= (data_rate << 3);

	if (noise_mode == MMA8452_NOISE_MODE_REDUCED)
		ctrl_reg1 |= MMA8452_CTRL_REG1_LNOISE;

	if (op_mode == MMA8452_OP_MODE_ACTIVE)
		ctrl_reg1 |= MMA8452_CTRL_REG1_ACTIVE;

	ret = mma8452_write_register(dev, i2c, MMA8452_CTRL_REG1, ctrl_reg1);
	if (ret)
		return ret;

	dev_info(dev,
		 "[%s]: MMA8452 initialized successfully (100 Hz, +/-2g, active mode)\n",
		 __func__);

	return 0;
}

static int mma8452_get_readings(struct device *dev,struct i2c_peripheral *i2c, MMA8452_t *readings)
{
	u8 status;
	u8 data[6];
	int ret;
	int i;

	if (!dev || !i2c || !readings)
		return -EINVAL;

	ret = mma8452_read_register(dev, i2c, MMA8452_STATUS_REG, &status);
	if (ret)
	{
		dev_err(dev, "[%s]: Failed to read STATUS: %d\n", __func__, ret);
		return ret;
	}

	if (!(status & BIT(3)))
		return -EAGAIN;

	if (status & BIT(7))
		dev_warn(dev, "[%s]: Previous acceleration data overwritten\n", __func__);

	for (i = 0; i < 6; i++)
	{
		ret = mma8452_read_register(dev, i2c, 0x01 + i, &data[i]);
		if (ret)
		{
			dev_err(dev, "[%s]: Failed to read register 0x%02x: %d\n", __func__, 0x01 + i, ret);
			return ret;
		}
	}

	readings->x = sign_extend32((((u16)data[0] << 8) | data[1]) >> 4, 11);
	readings->y = sign_extend32((((u16)data[2] << 8) | data[3]) >> 4, 11);
	readings->z = sign_extend32((((u16)data[4] << 8) | data[5]) >> 4, 11);

	dev_info(dev,
			 "[%s]: X=%d, Y=%d, Z=%d\n",
			 __func__,
			 readings->x,
			 readings->y,
			 readings->z);

	return 0;
}

static void mma8452_data_fetch_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);

	struct i2c_peripheral *i2c = container_of(dwork, struct i2c_peripheral, data_fetch_work);

	struct device *dev = i2c->dev;

	MMA8452_t readings = {0};
	int ret;

	ret = mma8452_get_readings(dev, i2c, &readings);

	if (ret == -EAGAIN)
	{
		dev_dbg(dev,
				"[%s]: No new acceleration data available\n",
				__func__);
	}
	else if (ret)
	{
		dev_err(dev,
				"[%s]: Failed to fetch readings: %d\n",
				__func__, ret);

		/*
		 * Stop polling after an I2C failure.
		 * The module must be reinitialized before
		 * further transfers.
		 */
		return;
	}

	schedule_delayed_work(
		&i2c->data_fetch_work,
		msecs_to_jiffies(MMA8452_FETCH_INTERVAL_MS));
}

static void mma8452_cancel_work(void *data)
{
	struct i2c_peripheral *i2c = data;

	cancel_delayed_work_sync(&i2c->data_fetch_work);
}

///////////////////// Driver callbacks /////////////////////

static int i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_peripheral *i2c;
	int ret;

	ret = i2c_init(pdev, &i2c);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, i2c);

	ret = mma8452_init(dev, i2c, MMA8542_I2C_ADDR);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&i2c->data_fetch_work, mma8452_data_fetch_work);

	ret = devm_add_action_or_reset(dev, mma8452_cancel_work, i2c);
	if (ret)
		return ret;

	schedule_delayed_work(&i2c->data_fetch_work, msecs_to_jiffies(MMA8452_FETCH_INTERVAL_MS));

	return 0;
}

static const struct of_device_id i2c_of_match[] = {
	{.compatible = "brcm,bcm2711-i2c"},
	{}};
MODULE_DEVICE_TABLE(of, i2c_of_match);

static struct platform_driver i2c_driver = {
	.probe = i2c_probe,
	.driver = {
		.name = "i2c-mma8452",
		.of_match_table = i2c_of_match,
	},
};

module_platform_driver(i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Devashish Lahariya");
MODULE_DESCRIPTION("Register-level BCM2711 I2C driver for MMA8452");
