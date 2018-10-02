#ifndef INCLUDE_MPU6050_H_
#define INCLUDE_MPU6050_H_

#include <linux/module.h>   /* Every Linux kernel module must include this head */
#include <linux/init.h>     /* Every Linux kernel module must include this head */
#include <linux/kernel.h>   /* printk() */
#include <linux/fs.h>       /* struct fops */
#include <linux/errno.h>    /* error codes */
#include <linux/cdev.h>     /* cdev_alloc()  */
#include <linux/ioport.h>   /* request_mem_region() */
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <cfg_type.h>
#include <cfg_gpio.h>
#include <mach/platform.h>
#include <mach/gpio_desc.h>
#include <mach/gpio.h>
#include <mach/soc.h>
#include <mach/iic.h>
#include <mach/regs-iic.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <linux/errno.h>


#define	MPU6050_SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU6050_CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	MPU6050_GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	MPU6050_ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44	
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48
#define	MPU6050_PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	MPU6050_WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	MPU6050_SLAVE_ADDRESS	0x69	//IIC写入时的地址字节数据，+1为读取

#define IO_LOW					0
#define IO_HIGH					1

typedef	struct mpu6050_t MPU6050;

struct mpu6050_hw_t {
	unsigned int io_collect[2];
	unsigned int io[0];
	struct i2c_driver *i2c_drv;
	struct i2c_client *i2c_clit;
	struct i2c_adapter *i2c_adper;
	unsigned char time[40];

	
};

struct mpu6050_master_t {
	int (*set_gyro_first)(MPU6050*, char);
	int (*set_accel_first)(MPU6050*, char);
	int (*set_lpf)(MPU6050*, char);
	int (*set_sample_rate)(MPU6050*, unsigned int);
	unsigned int (*get_x_accel)(MPU6050*);
	unsigned int (*get_y_accel)(MPU6050*);
	unsigned int (*get_z_accel)(MPU6050*);
	unsigned int (*get_x_gyro)(MPU6050*);
	unsigned int (*get_y_gyro)(MPU6050*);
	unsigned int (*get_z_gyro)(MPU6050*);
};

struct mpu6050_t {
	MPU6050 *this;
	struct mpu6050_hw_t *hw;
	struct mutex mutex_oled_time;
    struct spinlock	spinlock_oled_time;
	struct mpu6050_master_t *master;
	unsigned int majoy_id;
	unsigned int slave_address;
	unsigned int mem[50];
	char buffer[10];
};


#endif
