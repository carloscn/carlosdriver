/*
 * MPU6050 Sensor Module Linux Driver.
 *
 * (C) Copyright 2018
 * Carlos Wei, NWPU, <weihaochen@mail.nwpu.edu.cn>

 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * 
 */
 

#include "mpu6050.h"

/**
* \brief:    I2C connector.
*			
*     MPU6050 / SLAVE   	    SP6818/ MASTER
*     _______________          _______________
*    |               |        |               |
*    |          SCL  |<-------|  I2C_SCL0     |
*    |          SDA  |<-------|  I2C_SDA0	  |
*    |          AD0  |<-------|  aLIVEGPIO5   |
*    |               |        |               |
*
* \detail:
*    1) use sp6818 platform I2C0 as master device.
*	 2) AD0 pin is i2c device address, the mpu6050 address details in mpu6050.h.
*
*/
#define             DRV_AUTHOR                  "Wei haochen <weihaochen@mltbns.com>"
#define             DRV_DESC                    "MPU6050_I2C / I2C 2.0 interface."
#define             DRV_NAME                    "MPU6050"

#define             DRIVER_MAJOR               233
#define             DRIVER_SIZE                0x0010
#define             MEM_CLEAR                  0x01
#define             DRIVER_MAGIC               'j'

MPU6050* mpu6050;

// my name is mpu6050
static const struct of_device_id sp6818_i2c_of_mach[] = 
{
	{ .name = DRV_NAME, },
	{ },
};

static struct i2c_board_info __initdata sp6818_mpu6050_board_info = {
	I2C_BOARD_INFO("mpu6050-i2c", MPU6050_SLAVE_ADDRESS),
	/*
		7-bit slave address.
	*/
	//.flags  = I2C_CLIENT_TEN,
	.irq	= -1,
};
	
static int 
__mpu6050_write_reg(MPU6050* this, char reg_addr, char reg_value)
{
	int ret;
	struct i2c_msg msg;
	char write_buffer[10];
	
	memset(write_buffer, 0, 10);
	write_buffer[0] = (char)reg_addr;
	write_buffer[1] = (char)reg_value;
	msg.addr = (this->hw->i2c_clit->addr);
	msg.flags = 0;
	msg.len = 2;
	msg.buf = &write_buffer[0];
	ret = i2c_transfer(this->hw->i2c_adper, &msg, 1);

	return ret;
}

static int 
__mpu6050_read_reg(MPU6050* this, char reg_addr)
{

	struct i2c_msg msg[2];
	char write_buffer[10];
	int ret, i;

	memset(write_buffer, 0, 10);
	memset(this->buffer, 0, 10);
	write_buffer[0] = (char)reg_addr;
	msg[0].addr = (this->hw->i2c_clit->addr);
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &write_buffer[0];
	msg[1].addr = (this->hw->i2c_clit->addr);
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &this->buffer[0];
	
	ret = i2c_transfer(this->hw->i2c_adper, &msg, 2);
}

static unsigned int 
__mpu6050_merge_data(MPU6050* this, char reg_address)
{
	char h,l;

	__mpu6050_read_reg(this, reg_address);
	h = this->buffer[0];
	__mpu6050_read_reg(this, reg_address + 1);
	l = this->buffer[0];

	return (h << 8) + l;
}


static int 
__mpu6050_hw_init(MPU6050 *this, unsigned int address)
{
	int ret;
	struct i2c_client *client;
	struct i2c_adapter *adapter;

	printk(DRV_NAME "\tinit i2c adapter.\n");
	adapter = i2c_get_adapter(0);
	if (!adapter) {
		ret = -ENXIO;
		printk(DRV_NAME "\terror: %d : init i2c adapter failed.\n", ret);
		return ret;
	}
	strlcpy(adapter->name, "nxp_i2c",sizeof(adapter->name));
	printk(DRV_NAME "\tadd %s adapter successful.\n", adapter->name);
	printk(DRV_NAME "\tinit i2c client. \n");
	client = i2c_new_device(adapter, &sp6818_mpu6050_board_info);
	printk(DRV_NAME "\tclient address is %d bits\n", client->flags & I2C_CLIENT_TEN ? 10 : 7);
	if (!client) {
		ret = -ENXIO;
		printk(DRV_NAME "\terror: %d : init i2c client failed.\n", ret);
		return ret;
	}
	printk(DRV_NAME "\tadd %s client successful.\n", client->name);
	printk(DRV_NAME "\tinfo:\n");
	printk(DRV_NAME "\tclient addr: %x\n", client->addr);
	printk(DRV_NAME "\tclient name: %s\n", client->name);
	printk(DRV_NAME "\tadapter name: %s\n", client->adapter->name);
	this->hw->i2c_adper = adapter;
	this->hw->i2c_clit = client;
	this->hw->io_collect[0] = PAD_GPIO_ALV + 5;
	this->hw->io_collect[1] = PAD_GPIO_ALV + 4;

	return 0;
	
}
static int 
__mpu6050_init(MPU6050 *this)
{	
	int ret = 0;
	char id = 0;

	ret += __mpu6050_write_reg(this, MPU6050_PWR_MGMT_1, 0x80);
	mdelay(10);
	ret += __mpu6050_write_reg(this, MPU6050_PWR_MGMT_1, 0x00);
	ret += __mpu6050_write_reg(this, MPU6050_SMPLRT_DIV, 0x07);
	ret += __mpu6050_write_reg(this, MPU6050_CONFIG, 0x06);
	ret += __mpu6050_write_reg(this, MPU6050_GYRO_CONFIG, 0x18);
	ret += __mpu6050_write_reg(this, MPU6050_ACCEL_CONFIG, 0x01);
	memset(this->buffer, 0, 10);
	ret += __mpu6050_read_reg(this, MPU6050_WHO_AM_I);
	id = this->buffer[0];
	
	if (id != 0x68) {
		ret = -7;
		printk(DRV_NAME "\tDevice id check failed!\n");
	}
	return ret;
}

static int 
__mpu6050_reset(MPU6050 *this)
{
	return 0;
}

static int
__mpu6050_remove(MPU6050 *this)
{
	printk(DRV_NAME "\tRelease I2c adapter.\n");
	i2c_put_adapter(this->hw->i2c_adper);
	printk(DRV_NAME "\tRelease I2c client.\n");
	i2c_unregister_device(this->hw->i2c_clit);

	return 0;
}


static inline int 
mpu6050_module_set_gyro_first(MPU6050* this, char first)
{
	return __mpu6050_write_reg(this, MPU6050_GYRO_CONFIG, first << 3);
}

static inline int 
mpu6050_module_set_accel_first(MPU6050* this, char first)
{
	return __mpu6050_write_reg(this, MPU6050_ACCEL_CONFIG, first << 3);
}

static inline int 
mpu6050_module_set_lpf(MPU6050* this, char lpf)
{
	char data = 0;
	
	if (lpf > 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;

	return __mpu6050_write_reg(this, MPU6050_CONFIG, data);
}

static inline int
mpu6050_module_set_sample_rate(MPU6050* this, unsigned int rate)
{
	char data;
	int ret;
	
	if (rate > 1000)
		rate = 1000;
	if( rate < 4)
		rate = 4;
	data = 1000/rate - 1;
	ret = __mpu6050_write_reg(this, MPU6050_SMPLRT_DIV, data);

	return mpu6050_module_set_lpf(this, rate/2);
}

static inline unsigned int 
mpu6050_module_get_x_accel(MPU6050* this)
{
	return __mpu6050_merge_data(this, MPU6050_ACCEL_XOUT_H);
}

static inline unsigned int 
mpu6050_module_get_y_accel(MPU6050* this)
{
	return __mpu6050_merge_data(this, MPU6050_ACCEL_YOUT_H);
}

static inline unsigned int 
mpu6050_module_get_z_accel(MPU6050* this)
{
	return __mpu6050_merge_data(this, MPU6050_ACCEL_ZOUT_H);
}

static inline unsigned int 
mpu6050_module_get_x_gyro(MPU6050* this)
{
	return __mpu6050_merge_data(this, MPU6050_GYRO_XOUT_H);
}

static inline unsigned int 
mpu6050_module_get_y_gyro(MPU6050* this)
{
	return __mpu6050_merge_data(this, MPU6050_GYRO_YOUT_H);
}

static inline unsigned int 
mpu6050_module_get_z_gyro(MPU6050* this)
{
	return __mpu6050_merge_data(this, MPU6050_GYRO_ZOUT_H);
}

static MPU6050* mpu6050_new_dev(unsigned int addr)
{
	MPU6050* dev = (MPU6050*)kmalloc(sizeof(MPU6050),GFP_ATOMIC);
	dev->hw = (struct mpu6050_hw_t*)kmalloc(sizeof(struct mpu6050_hw_t),GFP_ATOMIC);
	dev->master = (struct mpu6050_master_t *)kmalloc(sizeof(struct mpu6050_hw_t),GFP_ATOMIC);
	dev->master->set_accel_first = &mpu6050_module_set_accel_first;
	dev->master->set_gyro_first = &mpu6050_module_set_gyro_first;
	dev->master->set_lpf = &mpu6050_module_set_lpf;
	dev->master->set_sample_rate = &mpu6050_module_set_sample_rate;
	dev->master->get_x_accel = &mpu6050_module_get_x_accel;
	dev->master->get_y_accel = &mpu6050_module_get_y_accel;
	dev->master->get_z_accel = &mpu6050_module_get_z_accel;
	dev->master->get_x_gyro = &mpu6050_module_get_x_gyro;
	dev->master->get_y_gyro = &mpu6050_module_get_y_gyro;
	dev->master->get_z_gyro = &mpu6050_module_get_z_gyro;

	return dev;
}


static size_t 
mpu6050_driver_write(struct file* filp, 
								 const char __user* buffer, 
								 size_t size, loff_t* f_pos) 
{
    unsigned long   p       =   *f_pos;
    unsigned int    count   =   size;
    int             ret     =   0;
	char 			mem[100];
	
    if (copy_from_user(mem + p, buffer, count)) {
        ret =   -EFAULT;
    }else {
        *f_pos += count;
        ret =   count;
        printk(DRV_NAME "\twrite %u bytes from %lu\n", count, p);
    }
	
    return ret;
}

static size_t 
mpu6050_driver_read(struct file* filp, 
								const char __user* buffer, 
								size_t size, loff_t* f_pos) 
{
    unsigned long p = *f_pos;
    unsigned int count = size;
    int ret = 0;
    int temp;
	char mem[20];
	unsigned int read_buffer[10];
	int i;
	
	read_buffer[0] = mpu6050->master->get_x_accel(mpu6050);
	read_buffer[1] = mpu6050->master->get_y_accel(mpu6050);
	read_buffer[2] = mpu6050->master->get_z_accel(mpu6050);
	read_buffer[3] = mpu6050->master->get_x_gyro(mpu6050);
	read_buffer[4] = mpu6050->master->get_y_gyro(mpu6050);
	read_buffer[5] = mpu6050->master->get_z_gyro(mpu6050);

	for (i = 0; i < 6; i ++) {
		mem[2*i] = (char)(read_buffer[i] & 0xFF);
		mem[2*i + 1] = (char)(read_buffer[i] >> 8) &0xFF;
	} 
	
    if (copy_to_user(buffer, mem + p, count)) {
        ret =   -EFAULT;
    }else {
        *f_pos += count;
        ret =   count;
		
        printk(DRV_NAME "\tread %u bytes from %lu\n", count, p);
        printk(DRV_NAME "\tread: %d \n", *buffer);
    }
	
    return ret;

}

static int	
mpu6050_driver_ioctl( struct file* filp, 
								unsigned int cmd, 
								unsigned long arg)
{ 
	return 0;
}

static int 
mpu6050_driver_open(struct inode* node, struct file* filp) 
{
	return 0;
}

static struct file_operations mpu6050_fops = 
{
        .owner              =   THIS_MODULE,
        .open               =   mpu6050_driver_open,
        .read               =   mpu6050_driver_read,
        .write              =   mpu6050_driver_write,
        .unlocked_ioctl     =   mpu6050_driver_ioctl,
};

static struct miscdevice mpu6050_miscdev = 
{
        .name               =   DRV_NAME,
        .fops               =   &mpu6050_fops,
        .nodename           =   DRV_NAME,
};


static void	__init mpu6050_driver_init(void)
{
	int ret,i;
	unsigned int reb = 0;

	printk(DRV_NAME "\tApply mem for MPU6050.\n");
	mpu6050 = mpu6050_new_dev(MPU6050_SLAVE_ADDRESS);
    if (!mpu6050) {
        ret = -ENOMEM;
        printk(DRV_NAME "\tMPU6050 new device failed!\n");
        kfree(mpu6050);
		return ret;
    }
	ret = misc_register(&mpu6050_miscdev);
	if (ret < 0 ) {
		printk(DRV_NAME "\tRegister Char Driver Failed. ret = %d\n", ret);
		kfree(mpu6050);
		return ret;
	}
	printk(DRV_NAME "\tgenerated a driver nod : /dev/MPU6050 \n");
	printk(DRV_NAME "\tInit hardware...\n");
	ret = __mpu6050_hw_init(mpu6050, MPU6050_SLAVE_ADDRESS);
	if (ret < 0) {
		kfree(mpu6050);
		return ret;
	}
	for (i= 0; i < ARRAY_SIZE(mpu6050->hw->io_collect); i ++) {
		ret = gpio_request(mpu6050->hw->io_collect[i], "MPU6050 GPIO");
		if (ret < 0) {
			kfree(mpu6050);
			misc_deregister(&mpu6050_miscdev);
			printk(DRV_NAME "\trequest gpio %d for mpu6050 failed, ret = %d\n", mpu6050->hw->io_collect[i], ret);
			return ret;
		} else {
		    printk(DRV_NAME "\trequest gpio %d for  mpu6050 set succussful, ret = %d\n", mpu6050->hw->io_collect[i], ret);
		}
		gpio_direction_output(mpu6050->hw->io_collect[i], 1);
    	gpio_set_value(mpu6050->hw->io_collect[i], IO_HIGH);
	}
	printk(DRV_NAME "\tInit spin lock and mutex...\n");
    spin_lock_init(&mpu6050->spinlock_oled_time);
    mutex_init(&mpu6050->mutex_oled_time);
	printk(DRV_NAME "\tInit module...\n");
	ret = __mpu6050_init(mpu6050);
	if (ret > 0) 
		printk(DRV_NAME "\tDevice %d init successful.\n", ret);
	
}
module_init(mpu6050_driver_init);

static void __exit mpu6050_driver_exit(void)
{
	int i;
	
	printk(DRV_NAME "\tRelease GPIOs.\n");
	for (i = 0; i < ARRAY_SIZE(mpu6050->hw->io_collect); i ++) 
		gpio_free(mpu6050->hw->io_collect[i]);
	printk(DRV_NAME "\tRelease Driver nod.\n");
	misc_deregister(&mpu6050_miscdev);
	printk(DRV_NAME "\tRelease the mpu6050 space.\n");
	__mpu6050_remove(mpu6050);
	kfree(mpu6050);
}
module_exit(mpu6050_driver_exit);


MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");
