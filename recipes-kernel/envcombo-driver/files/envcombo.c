// SPDX-License-Identifier: GPL-2.0
/*
 * envcombo.c - IIO driver for the ENV-COMBO sensor (ambient light channel)
 *
 * Only the ALS (IIO_LIGHT) functionality of the ENV-COMBO is implemented;
 * the temperature and humidity channels are out of scope for this driver.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>

#define ENVCOMBO_REG_WHO_AM_I	0x00
#define ENVCOMBO_REG_ALS_MSB	0x04
#define ENVCOMBO_REG_CFG	0x06
#define ENVCOMBO_REG_INT_CFG	0x07
#define ENVCOMBO_REG_STATUS	0x0C
#define ENVCOMBO_REG_CAL_ALS_GAIN	0x10
#define ENVCOMBO_REG_CAL_ALS_TIME	0x11
#define ENVCOMBO_REG_PWR_MODE	0x12

#define ENVCOMBO_WHO_AM_I_VAL	0xEB

#define ENVCOMBO_CFG_ALS_EN		BIT(7)
#define ENVCOMBO_CFG_ALS_GAIN_MASK	GENMASK(4, 3)
#define ENVCOMBO_CFG_ALS_TIME_MASK	GENMASK(2, 1)

#define ENVCOMBO_INT_CFG_EN	BIT(7)
#define ENVCOMBO_INT_CFG_LATCH	BIT(6)

#define ENVCOMBO_STATUS_ALS_RDY	BIT(3)

#define ENVCOMBO_PWR_MODE_MASK	GENMASK(1, 0)
#define ENVCOMBO_PWR_SLEEP	0x01
#define ENVCOMBO_PWR_ONE_SHOT	0x02
#define ENVCOMBO_PWR_CONTINUOUS	0x03

#define ENVCOMBO_RAW_READ_TIMEOUT_MS	500

/* Defaults written to CFG[4:3]/[2:1] at probe: gain x1, integration time 200ms */
#define ENVCOMBO_DEFAULT_GAIN_IDX	0
#define ENVCOMBO_DEFAULT_TIME_IDX	2

static const unsigned int envcombo_als_gain_table[] = { 1, 4, 16, 64 };
static const unsigned int envcombo_als_time_table_us[] = {
	50000, 100000, 200000, 400000,
};

/* IIO_VAL_INT_PLUS_MICRO available list: (integer, micro) pairs */
static const int envcombo_als_time_avail[] = {
	0, 50000,
	0, 100000,
	0, 200000,
	0, 400000,
};

struct envcombo_data {
	struct regmap *regmap;
	struct mutex lock;
	struct completion als_done;

	u8 als_gain_idx;
	u8 als_time_idx;
	u8 calib_again;
	u8 calib_atime;

	bool ev_en_rising;
	bool ev_en_falling;
	bool buffer_en;
};

static const struct iio_chan_spec envcombo_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				       BIT(IIO_CHAN_INFO_SCALE) |
				       BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				       BIT(IIO_CHAN_INFO_INT_TIME),
		.info_mask_separate_available =
				       BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				       BIT(IIO_CHAN_INFO_INT_TIME),
	},
};

/* Caller must hold data->lock. */
static int envcombo_update_power_mode(struct envcombo_data *data)
{
	bool active = data->buffer_en || data->ev_en_rising ||
		      data->ev_en_falling;

	return regmap_update_bits(data->regmap, ENVCOMBO_REG_PWR_MODE,
				   ENVCOMBO_PWR_MODE_MASK,
				   active ? ENVCOMBO_PWR_CONTINUOUS :
					    ENVCOMBO_PWR_SLEEP);
}

static int envcombo_read_als_raw(struct envcombo_data *data, int *val)
{
	u8 buf[2];
	int ret;

	mutex_lock(&data->lock);

	if (!data->buffer_en && !data->ev_en_rising && !data->ev_en_falling) {
		reinit_completion(&data->als_done);

		ret = regmap_write(data->regmap, ENVCOMBO_REG_PWR_MODE,
				    ENVCOMBO_PWR_ONE_SHOT);
		if (ret)
			goto out_unlock;

		if (!wait_for_completion_timeout(&data->als_done,
				msecs_to_jiffies(ENVCOMBO_RAW_READ_TIMEOUT_MS))) {
			ret = -ETIMEDOUT;
			goto out_unlock;
		}
	}

	ret = regmap_bulk_read(data->regmap, ENVCOMBO_REG_ALS_MSB, buf, 2);
	if (ret)
		goto out_unlock;

	*val = ((int)buf[0] << 8) | buf[1];
	ret = 0;

out_unlock:
	mutex_unlock(&data->lock);
	return ret;
}

static int envcombo_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct envcombo_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return envcombo_read_als_raw(data, val) ?: IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info envcombo_info = {
	.read_raw = envcombo_read_raw,
};

static irqreturn_t envcombo_irq_thread(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct envcombo_data *data = iio_priv(indio_dev);
	unsigned int status;
	int ret;

	ret = regmap_read(data->regmap, ENVCOMBO_REG_STATUS, &status);
	if (ret)
		return IRQ_NONE;

	if (status & ENVCOMBO_STATUS_ALS_RDY)
		complete(&data->als_done);

	return IRQ_HANDLED;
}

static const struct regmap_config envcombo_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ENVCOMBO_REG_PWR_MODE,
	.cache_type = REGCACHE_NONE,
};

static int envcombo_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct envcombo_data *data;
	struct iio_dev *indio_dev;
	unsigned int val;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	mutex_init(&data->lock);
	init_completion(&data->als_done);

	data->regmap = devm_regmap_init_i2c(client, &envcombo_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	ret = regmap_read(data->regmap, ENVCOMBO_REG_WHO_AM_I, &val);
	if (ret)
		return ret;
	if (val != ENVCOMBO_WHO_AM_I_VAL)
		return -ENODEV;

	indio_dev->name = "envcombo";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &envcombo_info;
	indio_dev->channels = envcombo_channels;
	indio_dev->num_channels = ARRAY_SIZE(envcombo_channels);

	data->als_gain_idx = ENVCOMBO_DEFAULT_GAIN_IDX;
	data->als_time_idx = ENVCOMBO_DEFAULT_TIME_IDX;

	ret = regmap_write(data->regmap, ENVCOMBO_REG_CFG,
			    ENVCOMBO_CFG_ALS_EN |
			    FIELD_PREP(ENVCOMBO_CFG_ALS_GAIN_MASK, data->als_gain_idx) |
			    FIELD_PREP(ENVCOMBO_CFG_ALS_TIME_MASK, data->als_time_idx));
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, ENVCOMBO_REG_INT_CFG,
			    ENVCOMBO_INT_CFG_EN | ENVCOMBO_INT_CFG_LATCH);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					 envcombo_irq_thread,
					 IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					 "envcombo", indio_dev);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id envcombo_id[] = {
	{ "envcombo" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, envcombo_id);

static struct i2c_driver envcombo_driver = {
	.driver = {
		.name = "envcombo",
	},
	.probe = envcombo_probe,
	.id_table = envcombo_id,
};
module_i2c_driver(envcombo_driver);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("ENV-COMBO IIO ambient light sensor driver");
MODULE_LICENSE("GPL");
