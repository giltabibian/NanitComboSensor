// SPDX-License-Identifier: GPL-2.0
/*
 * envcombo.c - IIO driver for the ENV-COMBO sensor
 *
 * TODO: Implement your driver here.
 */

#include <linux/module.h>
#include <linux/i2c.h>

static int envcombo_probe(struct i2c_client *client)
{
	return 0;
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
MODULE_DESCRIPTION("ENV-COMBO IIO temperature/humidity/light sensor driver");
MODULE_LICENSE("GPL");
