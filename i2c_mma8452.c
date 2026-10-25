
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/slab.h>

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

	if (!buf || !len || len > 16)
		return -EINVAL;

	i2c_set_data_length(i2c, len);

	// Clear FIFO while preserving controller enable.
	i2c_write_register(i2c, BSC_C_REG, BIT(BSC_C_I2CEN) | (3U << BSC_C_CLEAR));

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

#define MMA8452_STATUS_REG		0x00
#define MMA8452_SYSMOD_REG		0x0B
#define MMA8452_WHO_AM_I_REG 	0x0D
#define MMA8452_CTRL_REG1		0x2A
#define MMA8452_CTRL_REG2		0x2B
#define MMA8452_CTRL_REG3		0x2C
#define MMA8452_CTRL_REG4		0x2D
#define MMA8452_CTRL_REG5		0x2E

#define MMA8542_I2C_ADDR 		0x1C
#define MMA8542_DEVICE_ID		0x2A

typedef enum
{
	MMA8452_SYS_MODE_STANDBY = 0,
	MMA8452_SYS_MODE_WAKE,
	MMA8452_SYS_MODE_SLEEP,
}eMMA8452_SYS_MODE;

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

///////////////////// Driver callbacks /////////////////////

static int i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_peripheral *i2c;
	u8 mma8452_device_id = 0;
	int ret;
	eMMA8452_SYS_MODE current_sys_mode = MMA8452_SYS_MODE_STANDBY;

	ret = i2c_init(pdev, &i2c);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, i2c);

	i2c_set_slave_address(i2c, MMA8542_I2C_ADDR);

	// Verify the Device ID.
	ret = mma8452_read_register(dev, i2c, MMA8452_WHO_AM_I_REG, &mma8452_device_id);
	if (ret)
		return ret;

	dev_info(dev, "[%s]: MMA8452 Device ID: 0x%02X\n", __func__, mma8452_device_id);

	if (mma8452_device_id != MMA8542_DEVICE_ID)
	{
		dev_warn(dev, "[%s]: Unexpected MMA8452 device ID\n", __func__);
		return -ENODEV;
	}

	dev_info(dev, "[%s]: MMA8452 detected successfully\n", __func__);

	// Get the current system mode.
	dev_info(dev, "[%s]: Fetching current system mode\n", __func__);

	ret = mma8452_get_system_mode(dev, i2c, &current_sys_mode);
	if (ret)
		return ret;

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
