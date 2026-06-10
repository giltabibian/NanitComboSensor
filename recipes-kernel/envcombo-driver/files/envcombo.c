// SPDX-License-Identifier: GPL-2.0
/*
 * envcombo.c - IIO driver for the ENV-COMBO sensor (ambient light channel)
 *
 * Only the ALS (IIO_LIGHT) functionality of the ENV-COMBO is implemented;
 * the temperature and humidity channels are out of scope for this driver.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>

#define ENVCOMBO_REG_WHO_AM_I	0x00
#define ENVCOMBO_REG_PWR_MODE	0x12
#define ENVCOMBO_MAX_REG	ENVCOMBO_REG_PWR_MODE

#define ENVCOMBO_WHO_AM_I_VAL	0xEB

struct envcombo_data {
	struct regmap *regmap;
};

static const struct iio_chan_spec envcombo_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static int envcombo_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	return -EOPNOTSUPP;
}

static const struct iio_info envcombo_info = {
	.read_raw = envcombo_read_raw,
};

static const struct regmap_config envcombo_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ENVCOMBO_MAX_REG,
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
