/* Silicon Laboratories Si70xx humidity and temperature sensor driver
 *
 * Copyright (C) 2014 Silicon Laboratories
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA
 *
 * This device driver supports the Si7006, Si7013, Si7020 and Si7021 sensor ICs.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>

/* Module documentation */
MODULE_DESCRIPTION("Silicon Laboratories Si70xx humidity and temperature sensor");
MODULE_AUTHOR("Quentin Stephenson <Quentin.Stephenson@silabs.com>");
MODULE_LICENSE("GPL");

/* Commands */
#define CMD_MEASURE_HUMIDITY_HOLD	0xE5
#define CMD_MEASURE_HUMIDITY_NO_HOLD	0xF5
#define CMD_MEASURE_TEMPERATURE_HOLD	0xE3
#define CMD_MEASURE_TEMPERATURE_NO_HOLD 0xF3
#define CMD_MEASURE_THERMISTOR_HOLD	0xEE
#define CMD_READ_PREVIOUS_TEMPERATURE	0xE0
#define CMD_RESET			0xFE
#define CMD_WRITE_REGISTER_1		0xE6
#define CMD_READ_REGISTER_1		0xE7
#define CMD_WRITE_REGISTER_2		0x50
#define CMD_READ_REGISTER_2		0x10
#define CMD_WRITE_REGISTER_3		0x51
#define CMD_READ_REGISTER_3		0x11
#define CMD_WRITE_COEFFICIENT		0xC5
#define CMD_READ_COEFFICIENT		0x84

/* User Register 1 */
#define REG1_RESOLUTION_MASK		0x81
#define REG1_RESOLUTION_H12_T14 	0x00
#define REG1_RESOLUTION_H08_T12 	0x01
#define REG1_RESOLUTION_H10_T13 	0x80
#define REG1_RESOLUTION_H11_T11 	0x81
#define REG1_LOW_VOLTAGE		0x40
#define REG1_ENABLE_HEATER		0x04

/* User Register 2 */
#define REG2_VOUT			0x01
#define REG2_VREF_VDD			0x02
#define REG2_VIN_BUFFERED		0x04
#define REG2_RESERVED			0x08
#define REG2_FAST_CONVERSION		0x10
#define REG2_MODE_CORRECTION		0x20
#define REG2_MODE_NO_HOLD		0x40

/* Device Identification */
#define ID_SAMPLE			0xFF
#define ID_SI7006			0x06
#define ID_SI7013			0x0D
#define ID_SI7020			0x14
#define ID_SI7021			0x15

/* Coefficients */
#define COEFFICIENT_BASE		0x82
#define COEFFICIENT_COUNT		9

/* Thermistor Correction Coefficients */
static struct {
   s16 input[COEFFICIENT_COUNT];
   u16 output[COEFFICIENT_COUNT];
   s16 slope[COEFFICIENT_COUNT];
} si70xx_coefficient_table = {
   /* input  */ {19535, 15154, 11187,  7982,  5592,  3895,  2721,  1916,  1367},
   /* output */ {11879, 15608, 19338, 23067, 26797, 30527, 34256, 37986, 41715},
   /* slope  */ { -218,  -241,	-298,  -400,  -563,  -813, -1186, -1739, -2513}
};

/* Global variables */
struct si70xx {
	struct mutex lock;
	struct device *dev;
	int device_id;
};


/*
 * si70xx_show_device_id() - show device ID in sysfs
 * @dev:  device
 * @attr: device attribute
 * @buf:  sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to device_id sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t si70xx_show_device_id(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si70xx *si70xx = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", si70xx->device_id);
}

/*
 * si70xx_measure() - Take a measurement
 * @client:  I2C client device
 * @command: Command for measuring temperature or humidity
 *
 * Returns the 16-bit value from the Si70xx or errno on error.
 */
static int si70xx_measure(struct i2c_client *client, u8 command)
{
	union {
		u8  byte[4];
		s32 value;
	} data;

	/* Measure the humidity or temperature value */
	data.value = i2c_smbus_read_word_data(client, command);
	if (data.value < 0)
		return data.value;

	/* Swap the bytes and clear the status bits */
	return ((data.byte[0] * 256) + data.byte[1]) & ~3;
}

/*
 * si70xx_show_temperature1() - show temperature measurement value in sysfs
 * @dev:  device
 * @attr: device attribute
 * @buf:  sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to temp1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t si70xx_show_temperature1(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si70xx *si70xx = i2c_get_clientdata(client);
	int value;
	int temperature;

	/* Measure the temperature value */
	mutex_lock(&si70xx->lock);
	value = si70xx_measure(client, CMD_MEASURE_TEMPERATURE_HOLD);
	mutex_unlock(&si70xx->lock);
	if (value < 0)
		return value;

	/* Convert the value to millidegrees Celsius */
	temperature = ((value*21965)>>13)-46850;

	return sprintf(buf, "%d\n", temperature);
}

/*
 * si70xx_show_temperature2() - show thermistor measurement value in sysfs
 * @dev:  device
 * @attr: device attribute
 * @buf:  sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to temp2_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t si70xx_show_temperature2(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si70xx *si70xx = i2c_get_clientdata(client);
	int value;
	int temperature;
	int error;

	mutex_lock(&si70xx->lock);

	/* Bias the thermistor and use the correction coefficients */
	error = i2c_smbus_write_byte_data(client, CMD_WRITE_REGISTER_2,
		REG2_VREF_VDD|REG2_VIN_BUFFERED|REG2_RESERVED|REG2_MODE_CORRECTION);
	if (error < 0) {
		mutex_unlock(&si70xx->lock);
		return error;
	}

	/* Measure the temperature value */
	value = si70xx_measure(client, CMD_MEASURE_THERMISTOR_HOLD);

	/* Unbias the thermistor and stop using the correction coefficients */
	error = i2c_smbus_write_byte_data(client, CMD_WRITE_REGISTER_2,
		REG2_VOUT|REG2_VREF_VDD|REG2_VIN_BUFFERED|REG2_RESERVED);

	mutex_unlock(&si70xx->lock);

	if (value < 0)
		return value;
	if (error < 0)
		return error;

	/* Convert the value to millidegrees Celsius */
	temperature = ((value*21965)>>13)-46850;

	return sprintf(buf, "%d\n", temperature);
}

/*
 * si70xx_show_humidity() - show humidity measurement value in sysfs
 * @dev:  device
 * @attr: device attribute
 * @buf:  sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to humidity1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t si70xx_show_humidity(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si70xx *si70xx = i2c_get_clientdata(client);
	int value;
	int humidity;

	/* Measure the humidity value */
	mutex_lock(&si70xx->lock);
	value = si70xx_measure(client, CMD_MEASURE_HUMIDITY_HOLD);
	mutex_unlock(&si70xx->lock);
	if (value < 0)
		return value;

	/* Convert the value to milli-percent (pcm) relative humidity */
	value = ((value*15625)>>13)-6000;

	/* Limit the humidity to valid values */
	if (value < 0)
		humidity = 0;
	else if (value > 100000)
		humidity = 100000;
	else
		humidity = value;

	return sprintf(buf, "%d\n", humidity);
}

/* Device attributes for sysfs */
static SENSOR_DEVICE_ATTR(device_id, S_IRUGO, si70xx_show_device_id,
	NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, si70xx_show_temperature1,
	NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, si70xx_show_temperature2,
	NULL, 0);
static SENSOR_DEVICE_ATTR(humidity1_input, S_IRUGO, si70xx_show_humidity,
	NULL, 0);

/* Attribute array */
static struct attribute *si70xx_attributes[5];

/* Attribute group */
static const struct attribute_group si70xx_attr_group = {
	.attrs = si70xx_attributes,
};

/*
 * si70xx_get_device_id() - Get the device ID from the device
 * @client: I2C client device
 * @id: pointer to device ID
 *
 * Returns 0 on success.
 */
static int si70xx_get_device_id(struct i2c_client *client, int *id)
{
	char buf[6];
	int  error;

	/* Put the 2-byte command into the buffer */
	buf[0] = 0xFC;
	buf[1] = 0xC9;

	/* Send the command */
	error = i2c_master_send(client, buf, 2);
	if (error < 0)
	       return error;

	/* Receive the 6-byte result */
	error = i2c_master_recv(client, buf, 6);
	if (error < 0)
		return error;

	/* Return the device ID */
	*id = buf[0];

	return 0;  /* Success */
}

/*
 * si70xx_read_coefficient() - Read thermistor correction coefficient from Si7013
 * @client: I2C client device
 * @address: coefficient address
 * @coefficient: pointer to the coefficient value
 *
 * Returns 0 on success.
 */
int si70xx_read_coefficient(struct i2c_client *client, u8 address, s16 *coefficient)
{
	int MSB,LSB;
	int error;

	/* Read the MSB from the coefficient address */
	error = i2c_smbus_write_byte_data(client, CMD_READ_COEFFICIENT, address);
	if (error < 0)
		return error;
	MSB = i2c_smbus_read_byte(client);
	if (MSB < 0)
		return MSB;

	/* Read the LSB from the following coefficient address */
	error = i2c_smbus_write_byte_data(client, CMD_READ_COEFFICIENT, address+1);
	if (error < 0)
		return error;
	LSB = i2c_smbus_read_byte(client);
	if (LSB < 0)
		return LSB;

	/* Combine the MSB and LSB */
	*coefficient = (MSB*256) + LSB;

	return 0;  /* Success */
}

/*
 * si70xx_write_coefficient() - Write thermistor correction coefficient to Si7013
 * @client: I2C client device
 * @address: coefficient address
 * @coefficient: coefficient value
 *
 * Returns 0 on success.
 */
int si70xx_write_coefficient(struct i2c_client *client, u8 address, s16 coefficient)
{
	u16 value;
	int error;

	/* Write the MSB to the coefficient address */
	value = (coefficient&0xFF00) + address;
	error = i2c_smbus_write_word_data(client, CMD_WRITE_COEFFICIENT, value);
	if (error < 0)
		return error;

	/* Write the LSB to the following coefficient address */
	value = (coefficient<<8) + address + 1;
	error = i2c_smbus_write_word_data(client, CMD_WRITE_COEFFICIENT, value);
	if (error < 0)
		return error;

	return 0;  /* Success */
}

/*
 * si70xx_write_coefficient_table() - Write thermistor coefficients to Si7013
 * @client: I2C client device
 */
void si70xx_write_coefficient_table(struct i2c_client *client)
{
	s16 coefficient;
	u8  address;
	int error;
	int x;

	/* If the thermistor correction coefficients are not programmed into the Si7013 */
	error = si70xx_read_coefficient(client, COEFFICIENT_BASE, &coefficient);
	if ((error == 0) && (coefficient == -1))
	{
		/* Start at the beginning of coefficient memory */
		address = COEFFICIENT_BASE;

		/* Write the "input" coefficients */
		for (x=0; x<COEFFICIENT_COUNT; x++)
		{
			error = si70xx_write_coefficient(client, address,
				si70xx_coefficient_table.input[x]);
			if (error < 0)
				return;
			address += 2;
		}

		/* Write the "output" coefficients */
		for (x=0; x<COEFFICIENT_COUNT; x++)
		{
			error = si70xx_write_coefficient(client, address,
				si70xx_coefficient_table.output[x]);
			if (error < 0)
				return;
			address += 2;
		}

		/* Write the "slope" coefficients */
		for (x=0; x<COEFFICIENT_COUNT; x++)
		{
			error = si70xx_write_coefficient(client, address,
				si70xx_coefficient_table.slope[x]);
			if (error < 0)
				return;
			address += 2;
		}

                /* Reset the Si7013 to activate the coefficients */
                i2c_smbus_write_byte(client, CMD_RESET);
	}
}

/*
 * si70xx_probe() - Verify that this is the correct driver for the device
 * @client: I2C client device
 * @dev_id: device ID
 *
 * Called by the I2C core when an entry in the ID table matches a
 * device's name.
 * Returns 0 on success.
 */
static int si70xx_probe(struct i2c_client *client,
	const struct i2c_device_id *dev_id)
{
	struct si70xx *si70xx;
	int error;
	printk("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
	/* Allocate memory for global variables */
	si70xx = kzalloc(sizeof(*si70xx), GFP_KERNEL);
	if (si70xx == NULL)
		return -ENOMEM;
	i2c_set_clientdata(client, si70xx);

printk("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");
	/* Initialize the mutex */
	mutex_init(&si70xx->lock);

	/* Get the device ID from the device */
	mutex_lock(&si70xx->lock);
	error = si70xx_get_device_id(client, &si70xx->device_id);
	mutex_unlock(&si70xx->lock);
	if (error < 0)
	{
		kfree(si70xx);
		return error;
	}
printk("ccccccccccccccccccccccccccccccccccccc\n");
	/* Validate the device ID */
	if ((si70xx->device_id != ID_SAMPLE) &&
	    (si70xx->device_id != ID_SI7006) &&
	    (si70xx->device_id != ID_SI7013) &&
	    (si70xx->device_id != ID_SI7020) &&
	    (si70xx->device_id != ID_SI7021)) {
		kfree(si70xx);
		return -ENODEV;
	}
printk("dddddddddddddddddddddddddddd\n");
	/* If Si7013 then write the thermistor coefficients to the Si7013 */
	if (si70xx->device_id == ID_SI7013) {
		mutex_lock(&si70xx->lock);
		si70xx_write_coefficient_table(client);
		mutex_unlock(&si70xx->lock);
	}
printk("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee\n");
	/* Initialize the attribute array */
	si70xx_attributes[0] = &sensor_dev_attr_device_id.dev_attr.attr;
	si70xx_attributes[1] = &sensor_dev_attr_temp1_input.dev_attr.attr;
	si70xx_attributes[2] = &sensor_dev_attr_humidity1_input.dev_attr.attr;
printk("ffffffffffffffffffffffffffffff\n");
	/* If Si7013 then add a second temperature attribute for thermistor */
	if (si70xx->device_id == ID_SI7013) {
		si70xx_attributes[3] = &sensor_dev_attr_temp2_input.dev_attr.attr;
		si70xx_attributes[4] = NULL;
	}
	else si70xx_attributes[3] = NULL;
printk("gggggggggggggggggggggggggggggg\n");
	/* Create the sysfs group */
	error = sysfs_create_group(&client->dev.kobj, &si70xx_attr_group);
	if (error)
	{
		kfree(si70xx);
		return error;
	}
printk("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh\n");
	/* Register with hwmon */
	si70xx->dev = hwmon_device_register(&client->dev);
	if (IS_ERR(si70xx->dev)) {
		error = PTR_ERR(si70xx->dev);
		sysfs_remove_group(&client->dev.kobj, &si70xx_attr_group);
		kfree(si70xx);
		return error;
	}
printk("iiiiiiiiiiiiiiiiiiiiiiiiiiii\n");
	return 0;  /* Success */
}

/*
 * si70xx_remove() - Remove this driver from the device
 * @client: I2C client device
 */
static int si70xx_remove(struct i2c_client *client)
{
	struct si70xx *si70xx = i2c_get_clientdata(client);

	hwmon_device_unregister(si70xx->dev);
	sysfs_remove_group(&client->dev.kobj, &si70xx_attr_group);
	kfree(si70xx);

	return 0;  /* Success */
}

/* Device ID table */
static const struct i2c_device_id si70xx_id[] = {
	{"si70xx", 0},
	{}
};

/* Add the device ID to the module device table */
MODULE_DEVICE_TABLE(i2c, si70xx_id);

/* Driver structure */
static struct i2c_driver si70xx_driver = {
	.driver.name	= "si70xx",
	.probe		= si70xx_probe,
	.remove		= si70xx_remove,
	.id_table	= si70xx_id,
};

/*
 * si70xx_init() - Initialize the device driver
 */
static int __init si70xx_init(void)
{
	/* Register this I2C chip driver */
	return i2c_add_driver(&si70xx_driver);
}
module_init(si70xx_init);

/*
 * si70xx_exit() - Exit the device driver
 */
static void __exit si70xx_exit(void)
{
	/* Unregister this I2C chip driver */
	i2c_del_driver(&si70xx_driver);
}
module_exit(si70xx_exit);
