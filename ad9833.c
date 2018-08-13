/*
 * AD9833 of ADI driver code for Texas Instruments OMAPL138+
 *
 * Copyright (C) 2018 Wei Haochen
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
 * MULTIBEANS, NPU Youyi West Ave, Beilin District, Xi'an, China.
 */

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
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>


#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/da8xx.h>
#include <mach/mux.h>
#include <asm/uaccess.h>
#include <asm/div64.h>



#define				DRV_AUTHOR					"Wei haochen <weihaochen@mltbns.com>"
#define				DRV_DESC					"AD9833 on OMAPL138-ARM9"
#define				DRV_NAME					"AD9833-ADI"


#define				AD9833_SIZE					0x1000
#define				MEM_CLEAR					0x1
#define				AD9833_MAJOR				230
#define           	AD9833_REG_RESET            0x0100
#define           	AD9833_FREQ0_REG            0
#define             AD9833_FREQ1_REG            1

#define				AD9833_MAGIC				'k'
#define				CMD_PARA_FREQ				0x10
#define				CMD_PARA_PHASE				0x11
#define				CMD_PARA_TYPE				0x12

#define				CMD_TYPE_SIN				_IO( AD9833_MAGIC, 0)
#define				CMD_TYPE_TRI				_IO( AD9833_MAGIC, 1)
#define				CMD_TYPE_SQE				_IO( AD9833_MAGIC, 2)

#define				CMD_FREQ_SET(X)				_IO( CMD_PARA_FREQ, X)
#define				CMD_PHASE_SET(X)			_IO( CMD_PARA_PHASE, X )
#define				CMD_TYPE_SET(X)				_IO( CMD_PARA_TYPE,X )

#define				IO_HIGH						1
#define				IO_LOW						0

#define				AD9833_FSY_IO				GPIO_TO_PIN(0, 1)
#define				AD9833_CLK_IO				GPIO_TO_PIN(0, 5)
#define				AD9833_DAT_IO				GPIO_TO_PIN(0, 0)

#define				io_clk(x)					gpio_set_value( AD9833_CLK_IO,x )
#define				io_fsy(x)					gpio_set_value( AD9833_FSY_IO,x )
#define				io_dat(x)					gpio_set_value( AD9833_DAT_IO,x )


typedef		struct ad9833_t 	AD9833;

enum ad9833_wavetype_t{
	SIN,SQU,TRI
};


struct ad9833_hw_t {

	unsigned int clk;
	unsigned int sdi;
	unsigned int fsy;
};

struct ad9833_t {

	struct ad9833_hw_t hw;
	struct ad9833_t *self;
	enum ad9833_wavetype_t wave_type;

	struct	cdev	cdev;
	unsigned char	mem[ AD9833_SIZE ];

	unsigned int delay;

	void (*write_reg)	( AD9833 *self, unsigned int reg_value);
	void (*init_device)	( AD9833 *self );
	void (*set_wave_freq)( AD9833 *self , unsigned long freqs_data);
	void (*set_wave_type)( AD9833 *self, enum ad9833_wavetype_t wave_type );
	void (*set_wave_phase)( AD9833 *self, unsigned int phase );
	void (*set_wave_para)( AD9833 *self, unsigned long freqs_data, unsigned int phase, enum ad9833_wavetype_t wave_type );
};



static void ad9833_set_wave_type( AD9833 *dev, enum ad9833_wavetype_t wave_type );
static void ad9833_set_phase( AD9833 *dev, unsigned int phase_value );
static void ad9833_set_freq( AD9833 *dev, unsigned long freq );
static void ad9833_set_para( AD9833 *dev, unsigned long freqs_value, unsigned int phase_value, enum ad9833_wavetype_t wave_type );
static void ad9833_init_device( AD9833 *dev ) ;
static void ad9833_write_reg( AD9833 *dev, unsigned int reg_value );
static int 	ad9833_ioctl(struct file  *file, unsigned int cmd, unsigned long arg );
static void ad9833_dev_exit( void );

AD9833 *ad9833;
static int	ad9833_major	=	AD9833_MAJOR;

module_param( ad9833_major, int, S_IRUGO );


static const short	ad9833_gpios[] = {
		AD9833_FSY_IO,
		AD9833_CLK_IO,
		AD9833_DAT_IO,
		GPIO_TO_PIN(6, 1),
};

struct gpio_irq_desc {

	int irq;
	unsigned long flags;
	char *name;

} press_dev_desc = {

		IRQ_DA8XX_GPIO0,
		IRQ_TYPE_EDGE_FALLING,
		"sw6_push_button"

};

AD9833 *ad9833_dev_new()
{
	AD9833 *dev = (AD9833*)kcalloc(1, sizeof(AD9833), GFP_ATOMIC);

	dev->hw.fsy			  =	  AD9833_FSY_IO;
	dev->hw.sdi			  =   AD9833_DAT_IO;
	dev->hw.clk			  =	  AD9833_CLK_IO;

	dev->set_wave_para    =   &ad9833_set_para;
	dev->init_device      =   &ad9833_init_device;
	dev->write_reg        =   &ad9833_write_reg;
	dev->set_wave_freq    =   &ad9833_set_freq;
	dev->set_wave_phase	  =   &ad9833_set_phase;
	dev->set_wave_type    =   &ad9833_set_wave_type;
	dev->init_device( dev );

	return dev;
}
static int key_count = 0;
static irqreturn_t	ad9833_press_intHandle( int irq, void *dev_id )
{
	printk( DRV_NAME "\t press trigger!\n" );
	if( key_count == 0 )  {
	    ad9833->set_wave_type( ad9833, SIN );
	    printk( DRV_NAME "\tSet wave is SIN.\n" );
	}else if( key_count == 2 ) {
	    ad9833->set_wave_type( ad9833, TRI );
	    printk( DRV_NAME "\tSet wave is TRI.\n" );
	}else if( key_count == 4 ) {
	    ad9833->set_wave_type( ad9833, SQU );
	    printk( DRV_NAME "\tSet wave is SQU.\n" );
	}
	key_count ++;
	if( key_count >= 5 )
	    key_count = 0;
	    
	return	IRQ_RETVAL( IRQ_HANDLED );
}

static int 	ad9833_open( struct inode *inodes, struct file *filp )
{
	int ret;
	printk( DRV_NAME "\tAD9833 open function..\n" );

	return 0;
}


static int	ad9833_ioctl(struct file  *file,
						 unsigned int cmd, unsigned long arg )
{

	printk(DRV_NAME "\tRecv cmd: %u\n", cmd);
	printk(DRV_NAME "\tRecv arg: %lu\n", arg);
	switch( cmd ) {
	case CMD_TYPE_SIN:
		ad9833->set_wave_freq(ad9833, 1500);
		ad9833->set_wave_type(ad9833, SIN);
		printk( DRV_NAME " set wave is sine wave! arg = %lu\n" , arg );

		break;

	case CMD_TYPE_TRI:
		ad9833->set_wave_freq(ad9833, 1500);
		ad9833->set_wave_type(ad9833, TRI);
		printk( DRV_NAME " set wave is tri wave! arg = %lu\n" , arg );
		break;

	case CMD_TYPE_SQE:
		ad9833->set_wave_freq(ad9833, 1500);
		ad9833->set_wave_type(ad9833, SQU);
		printk( DRV_NAME " set wave is sw wave! arg = %lu\n" , arg );
		break;

	}
	return	0;
}

static void	ad9833_write_reg(AD9833 *dev,
							 unsigned int reg_value )
{
	unsigned short i;

	io_clk(IO_HIGH);
	io_fsy(IO_HIGH);
	ndelay(10);
	io_fsy(IO_LOW);

	for ( i = 0; i < 16; i++ ) {

		if ( reg_value & 0x8000 )
			io_dat(IO_HIGH);
		else
			io_dat(IO_LOW);
		ndelay(10);
		io_clk(IO_LOW);
		ndelay(10);
		io_clk(IO_HIGH);
		reg_value = reg_value << 1;
	}
	io_fsy(IO_HIGH);
}

static void	ad9833_init_device( AD9833 *dev )
{
	dev->write_reg( dev, AD9833_REG_RESET );
	dev->set_wave_para( dev,1500, 0 ,SIN );
}

static void ad9833_set_para( AD9833 *dev,
							unsigned long freqs_value, unsigned int phase_value,
							enum ad9833_wavetype_t wave_type )
{
	unsigned long dds_frequence_data;
	unsigned int dds_frequence_low;
	unsigned int dds_frequence_high;
	unsigned int phase_data;
	phase_data      =   phase_value | 0xC000;

	dds_frequence_data      =   freqs_value * 10;
	dds_frequence_low       =   dds_frequence_data & 0x3FFF;
	dds_frequence_low       |=  0x4000;
	dds_frequence_data      =   dds_frequence_data >> 14;
	dds_frequence_high      =   dds_frequence_data & 0x3FFF;
	dds_frequence_high      |=  0x4000;
	// 	reset device
	dev->write_reg( dev, 0x0110 );
	dev->write_reg( dev, 0x2100 );

	dev->write_reg( dev,dds_frequence_low );
	dev->write_reg( dev,dds_frequence_high );
	dev->write_reg( dev, phase_data );

	if( wave_type == TRI ) {
		dev->write_reg( dev, 0x2002 );
	}else if( wave_type == SQU ) {
		dev->write_reg(  dev, 0x2028);
	}else {
		dev->write_reg( dev, 0x2000 );
	}
}

static void
ad9833_set_freq( AD9833 *dev, unsigned long freq )
{
	unsigned long dds_frequence_data;
	unsigned long dds_frequence_low;
	unsigned long dds_frequence_high;

	dds_frequence_data      =   freq;
	dds_frequence_low       =   dds_frequence_data & 0x3FFF;
	dds_frequence_low       |=  0x4000;
	dds_frequence_data      =   dds_frequence_data >> 14;
	dds_frequence_high      =   dds_frequence_data & 0x3FFF;
	dds_frequence_high      |=  0x4000;

	dev->write_reg( dev, dds_frequence_low );
	dev->write_reg( dev, dds_frequence_high );

}

static void
ad9833_set_phase( AD9833 *dev, unsigned int phase_value )
{
	unsigned int phase_temp;
	phase_temp = phase_value | 0xC000;
	dev->write_reg( dev, phase_temp );
}

static void
ad9833_set_wave_type( AD9833 *dev, enum ad9833_wavetype_t wave_type )
{
	if( wave_type == TRI ) {
		dev->write_reg( dev, 0x2002 );
	}else if( wave_type == SQU ) {
		dev->write_reg(  dev, 0x2028);
	}else {
		dev->write_reg( dev, 0x2000 );
	}
}

static ssize_t
ad9833_driver_read( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos )
{
	unsigned long 	p		=	*f_pos;
	unsigned int 	count	=	size;
	int 			ret		=	0;

	if ( p >= AD9833_SIZE )
		return 0;
	if ( count > AD9833_SIZE - p )
		count = AD9833_SIZE - p;
	if ( copy_to_user( buffer, ad9833->mem + p, count) ) {
		ret	=	-EFAULT;
	}else {
		*f_pos += count;
		ret = 	count;
		printk( DRV_NAME "\tread %u bytes from %lu\n", count, p );
	}
	return ret;
}

static ssize_t
ad9833_driver_write( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos )
{
	unsigned long 	p		=	*f_pos;
	unsigned int 	count	=	size;
	int 			ret		=	0;

	if ( p >= AD9833_SIZE )
		return 0;
	if ( count > AD9833_SIZE - p )
		count = AD9833_SIZE - p;

	memset( ad9833->mem,0, AD9833_SIZE );

	if ( copy_from_user( ad9833->mem + p, buffer, count) ) {
		ret	=	-EFAULT;
	}else {
		*f_pos += count;
		ret = 	count;
		printk( DRV_NAME "\twrite %u bytes from %lu\n", count, p );
		printk( DRV_NAME "\tRecv: %s \n", ad9833->mem + p );
		printk( DRV_NAME "\tSet freq is: %d \n", simple_strtol(ad9833->mem + p,"str",0) );
		ad9833->set_wave_freq(ad9833, simple_strtol(ad9833->mem + p,"str",0) );
	}
	return ret;
}


static struct file_operations ad9833_fops = {

		.owner				=	THIS_MODULE,
		.open				=	ad9833_open,
		.read				=  	ad9833_driver_read,
		.write				=	ad9833_driver_write,
		.unlocked_ioctl  	=  	ad9833_ioctl,
};

static struct miscdevice ad9833_miscdev  = {

		.name				=	DRV_NAME,
		.fops				=	&ad9833_fops,
};

dev_t	devno;
static int __init ad9833_dev_init( void )
{
	int  	i,ret;
	int  	index_minor = 0;
	int 	mk_major;

	/*
	 * cdev alloc and release device code.
	 * */
	devno = MKDEV( ad9833_major, index_minor );
	mk_major	=	MKDEV(ad9833_major, 0);
	if( ad9833_major ) {
		ret = register_chrdev_region( devno, 1, DRV_NAME );
	}else {
		ret = alloc_chrdev_region( &devno, 0, 1, DRV_NAME );
		ad9833_major	=	MAJOR(devno);
	}
	if( ret < 0 ) {
		printk(DRV_NAME "\t cdev alloc space failed.\n");
		return ret;
	}
	/*
	 * AD9833 new device
	 * */
	printk( DRV_NAME "\tApply memory for AD9833.\n" );
	ad9833 = ad9833_dev_new();
	if( !ad9833 ) {
		ret = -ENOMEM;
		printk(DRV_NAME "\tad9833 new device failed!\n" );
		goto fail_malloc;
	}

	/*
	 * AD9833 init gpios.
	 * */
	printk( DRV_NAME "\tInititial GPIO\n" );

	for ( i = 0; i < ARRAY_SIZE( ad9833_gpios ); i ++ ) {
		ret	=	gpio_request( ad9833_gpios[i], "AD9833 GPIO" );
		if( ret ) {
			printk( DRV_NAME "\trequest gpio %d for AD9833 failed, ret = %d\n", ad9833_gpios[i],ret);
			return ret;
		}else {
			printk( DRV_NAME "\trequest gpio %d for AD9833 set succussful, ret = %d\n", ad9833_gpios[i],ret);
		}

	}
	for( i = 0; i < 3; i ++ ) {
		gpio_direction_output( ad9833_gpios[i],1 );
		gpio_set_value( ad9833_gpios[i],0 );
	}
    gpio_direction_output( ad9833_gpios[3], 0 );
    irq_set_irq_type(gpio_to_irq(ad9833_gpios[3]), IRQF_TRIGGER_LOW);
	/*
	 * cdev init.
	 * */ 
	cdev_init( &ad9833->cdev, &ad9833_fops );
	ad9833->cdev.owner	=	THIS_MODULE;
	ret = cdev_add( &ad9833->cdev, mk_major,1 );
	if( ret ) {
		printk( KERN_NOTICE "Error %d adding ad9833 %d", ret, 1 );
		unregister_chrdev_region( devno,1 );
		for( i = 0; i < ARRAY_SIZE(ad9833_gpios); i++) 
		    gpio_free( ad9833_gpios[i] );
        return ret;
	}
	/*
	 * interrupt apply
	 * */
	press_dev_desc.irq =  gpio_to_irq(ad9833_gpios[3]);
	ret =	request_irq( press_dev_desc.irq , &ad9833_press_intHandle, press_dev_desc.flags, press_dev_desc.name, (void*)0 );
	if( ret ) {
		printk( DRV_NAME "\terror %d: IRQ = %d number failed!\n",ret,gpio_to_irq(ad9833_gpios[3]) );
		ret = -EBUSY;
	    unregister_chrdev_region( devno,1 );
		for( i = 0; i < ARRAY_SIZE(ad9833_gpios); i++) 
		    gpio_free( ad9833_gpios[i] );
		kfree(ad9833);
        return ret;
	}
	printk( DRV_NAME "\tiqr apply ok!!\n" );
	//ret = misc_register( &ad9833_miscdev );
	printk( DRV_NAME "\tinitialized\n" );
	return 0;

	fail_malloc:
	unregister_chrdev_region( devno,1 );
	return ret;

}


static void __exit ad9833_dev_exit( void )
{
	int i;
	for( i = 0; i < ARRAY_SIZE(ad9833_gpios); i++) {
		gpio_free( ad9833_gpios[i] );
	}
	//misc_deregister( &ad9833_miscdev );
	unregister_chrdev_region( devno,1 );
	// release irq
	free_irq( press_dev_desc.irq, (void*)0 );

}

module_init( ad9833_dev_init );
module_exit( ad9833_dev_exit );


MODULE_AUTHOR( DRV_AUTHOR );
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");


