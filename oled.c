/*
 * OLED of Module for Samsung S5P6818 Device.
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
 * 
 * You can download the license on our Github. ->
 * -> https://github.com/lifimlt  <-
 * MULTIBEANS ORG. http://www.mltbns.com/
 * Wei Haochen.  Email: weihaochen@mltbns.com
 * NWPU, 127 Youyi West Ave, Beilin District, Xi'an China.
 */
 
#include "oled.h"

/**
* \brief    SPI connector.
*			
*       OLED / SLAVE   		    SP6818/ MASTER
*     _______________          _______________
*    |               |        |               |
*    |          SCK  |<-------|  SCLK0/GPIOC29|
*    |          SDA  |<-------|  MOSI0/GPIOC31|
*    |          RES  |<-------|  GPIOD0       |
*    |          DC   |<-------|  GPIOB30      |
*    |          CS   |<-------|  CS0/GPIOC30  |
*    |               |        |               |
*
*/

#define             DRV_AUTHOR                  "Wei haochen <weihaochen@mltbns.com>"
#define             DRV_DESC                    "OLED_SPI / SPI0 2.0 interface."
#define             DRV_NAME                    "OLED"

#define             DRIVER_MAJOR               232
#define             DRIVER_SIZE                0x0010
#define             MEM_CLEAR                  0x01
#define             DRIVER_MAGIC               'k'

OLED	*oled;

unsigned int oled_gpio_collect[] = 
{
	OLED_DC_IO,
	OLED_RES_IO,
};

static const struct spi_device_id oled_spi_id[] =
{
		{DRV_NAME, 1},
		{},
};
				  
static struct spi_driver sp6818_spi_driver = 
{
		.driver 			= 	
		{
				.name		=	"oled_spi",
				.bus		=	&spi_bus_type,
				.owner  	= 	THIS_MODULE,
		},
		.probe				=	oled_bus_spi_probe,
		.remove 			= 	__devexit_p(oled_bus_spi_remove),
		.suspend 			= 	oled_bus_spi_suspend,
		.id_table			=	oled_spi_id,
};

MODULE_DEVICE_TABLE( spi, oled_spi_id );
static struct spi_device sp6818_spi_device = 
{
		.mode				=	SPI_MODE_3,
		.bits_per_word		=	16,
		.chip_select		=	SPI_CS_HIGH,
		.max_speed_hz		=	100000,
};

static struct file_operations oled_fops = 
{
        .owner              =   THIS_MODULE,
        .open               =   oled_driver_open,
        .read               =   oled_driver_read,
        .write              =   oled_driver_write,
        .unlocked_ioctl     =   oled_driver_ioctl,
};

static struct miscdevice oled_miscdev = 
{
        .name               =   DRV_NAME,
        .fops               =   &oled_fops,
        .nodename           =   DRV_NAME,
};

static struct s3c64xx_spi_csinfo sp6818_csi = 
{
        .line       		= 	OLED_CS_IO,
        .set_level  		= 	gpio_set_value,
        .fb_delay   		= 	0x2,
};
		
struct spi_board_info sp6818_board_info = 
{
        .modalias       	= 	"oled",
        .platform_data  	= 	NULL,
        .max_speed_hz   	= 	10 * 1000 * 1000,
        .bus_num        	= 	0,
        .chip_select    	= 	2,
        .mode           	= 	SPI_MODE_3,
        .controller_data    = 	&sp6818_csi,
};

OLED* oled_dev_new( void )
{
	OLED* self = (OLED*)kmalloc(sizeof(OLED), GFP_ATOMIC);

	self->master 				= 	(struct oled_master_t*) \
								  	kmalloc( sizeof(struct oled_master_t) * 10, \
								  	GFP_ATOMIC);
	self->master->init			=	&oled_module_init;
	self->master->hw_init		=	&oled_module_hw_init;
	self->master->reset			=	&oled_module_reset;
	self->master->clear			=	&oled_module_clear;
	self->master->display_on	= 	&oled_module_display_on;
	self->master->display_off	=	&oled_module_display_off;
	self->master->write_byte	=	&oled_module_write_byte;
	self->master->setpos		=	&oled_module_set_pos;
	self->master->show_str		=	&oled_module_show_str;
	self->master->show_char		=	&oled_module_show_char;
	self->master->set_dc_high	=	&oled_module_dc_set_high;
	self->master->set_dc_low	=	&oled_module_dc_set_low;
	self->master->set_res_high	=	&oled_module_res_set_high;
	self->master->set_res_low	=	&oled_module_res_set_low;
		
	return self;
}

static const struct dev_pm_ops oled_ops = {


};

static void oled_module_dc_set_high( OLED *self)
{
    gpio_direction_output( self->hw.dc_io_num , IO_HIGH );
}

static void oled_module_dc_set_low( OLED *self )
{
	gpio_direction_output( self->hw.dc_io_num , IO_LOW );
}

static void oled_module_res_set_high( OLED *self )
{
	gpio_direction_output( self->hw.res_io_num, IO_HIGH );
}

static void oled_module_res_set_low( OLED *self )
{
	gpio_direction_output( self->hw.res_io_num, IO_LOW );
}

static int  oled_bus_spi_probe( struct spi_device *spi )
{
	int ret;

	printk( DRV_NAME "\tPROBE... SPI\n" );
	oled->hw.spi_dev	=	spi;
	spi_message_init( &oled->hw.spi_msg );	
	printk( DRV_NAME "\t spi register master->..\n" );

	return ret;
}

static int  oled_bus_spi_remove( struct spi_device *spi )
{
	int ret;

	

	return 0;
}

int oled_bus_spi_suspend( struct spi_device *spi, 
						  pm_message_t mesg )
{
	int ret;

	return ret;
}
		
static size_t oled_driver_write( struct file* filp, 
								 const char __user* buffer, 
								 size_t size, loff_t* loff ) 
{

	return 0;
}

static size_t oled_driver_read( struct file* filp, 
								const char __user* buffer, 
								size_t size, loff_t* loff ) 
{
	return 0;
}

static int	oled_driver_ioctl(  struct file* filp, 
								unsigned int cmd, 
								unsigned long arg )
{ 
	return 0;
}

static int oled_driver_open( struct inode* node, struct file* filp ) 
{
	return 0;
}
#define	CONFIG_SPI_SLSI_PORT0 0
static void	__init oled_driver_init( void )
{
	int ret,i;

	for ( i= 0; i < ARRAY_SIZE( oled_gpio_collect ); i ++ ) {
		ret =  gpio_request( oled_gpio_collect[i], "OLED GPIO" );
		if ( ret ) {
			printk( DRV_NAME "\trequest gpio %d for OLED failed, ret = %d\n", oled_gpio_collect[i], ret);
			return ret;
		} else {
		    printk( DRV_NAME "\trequest gpio %d for OLED set succussful, ret = %d\n", oled_gpio_collect[i], ret);
		}
		gpio_direction_output( oled_gpio_collect[i], 1 );
    	gpio_set_value( oled_gpio_collect[i], IO_LOW );
	}
	printk( DRV_NAME "\tApply mem for OLED.\n" );
	oled = oled_dev_new();
    if( !oled ) {
        ret = -ENOMEM;
        printk(DRV_NAME "\tOLED new device failed!\n" );
        kfree( oled );
    }
	printk( DRV_NAME "\tgenerated a driver nod...\n" );
	ret = misc_register( &oled_miscdev );
	printk( DRV_NAME "\tInit spin lock and mutex...\n" );
    spin_lock_init(&oled->spinlock_oled_time);
    mutex_init(&oled->mutex_oled_time);
	oled->master->hw_init( oled );
	oled->master->init( oled );
	printk( DRV_NAME "\tShow Hello World on OLED\n" );
	oled->master->show_str( oled, 0, 0, "Hello World!\n" );	
}

static void __exit oled_driver_exit( void )
{
	int i;

	for ( i = 0; i < ARRAY_SIZE( oled_gpio_collect ); i ++ ) 
		gpio_free( oled_gpio_collect[i] );
	misc_deregister(&oled_miscdev);
	spi_unregister_driver(oled->hw.spi_drv);
	spi_unregister_device(oled->hw.spi_dev);
	kfree(oled->hw.spi_master_bus);
	kfree(oled->master);
	kfree(oled);
}

static void oled_module_hw_init( OLED *self )
{
	int ret,i;
	struct spi_master *master;
	struct spi_device *spi;

	self->hw.res_io_num = OLED_RES_IO;
	self->hw.dc_io_num	= OLED_DC_IO;
	printk( DRV_NAME "\tregister spi driver...\n" );
	self->hw.spi_drv = &sp6818_spi_driver;
	ret = spi_register_driver( self->hw.spi_drv );
	if ( ret < 0 ) {
		printk( DRV_NAME "\terror: spi driver register failed" );
	}
	printk( DRV_NAME "\tmaster blind spi bus.\n" );
	master = spi_busnum_to_master( 0 );
	master->num_chipselect = 4;
	if ( !master ) {
		printk( DRV_NAME "\terror: master blind spi bus.\n" );
		ret = -ENODEV;
		return ret;
	}
	printk( DRV_NAME "\tnew spi device...\n" );
	spi =	spi_new_device( master, &sp6818_board_info );
	if ( !spi ) {
		printk( DRV_NAME "\terror: spi occupy.\n" );		
		return -EBUSY;
	}
	self->hw.spi_master_bus	= master;
	self->hw.spi_dev = spi;
	printk( DRV_NAME "\thw init succussful...\n" );
}

static void oled_module_init( OLED *self )
{
	int i;
	unsigned int init_rom[] = 
	{
		0xae, 0x00, 0x10, 0x40, 0x81, 0xcf, 0xa1, 0xc8,
		0xa6, 0xa8, 0x3f, 0xd3, 0x00, 0xd5, 0x80, 0xd9,
		0xf1, 0xda, 0x12, 0xdb, 0x40, 0x20, 0x02, 0x02, 
		0x8d, 0x14, 0xA4, 0xA6, 0xAF
	};
		
	self->master->reset( self );
	for ( i = 0; i < ARRAY_SIZE( init_rom ); i ++ ) 
		self->master->write_byte( self, init_rom[i], ENUM_WRITE_TYPE_CMD );
	self->master->clear( self );
	self->master->setpos( self, 0, 0);
}


static void oled_module_reset( OLED *self )
{
	self->master->set_res_high( self );
	mdelay( 100 );
	self->master->set_res_low( self );
	mdelay( 100 );
	self->master->set_res_high( self );
	mdelay( 100 );
}

static void oled_module_clear( OLED *self )
{
	int i, n;
	
    for ( i = 0; i < 8; ++i ) {
        self->master->write_byte( self, 0xb0 + i, ENUM_WRITE_TYPE_CMD );
        self->master->write_byte( self, 0x00, ENUM_WRITE_TYPE_CMD );
        self->master->write_byte( self, 0x10, ENUM_WRITE_TYPE_CMD );
        for (n = 0; n < 128; ++n) 
            self->master->write_byte( self, 0, ENUM_WRITE_TYPE_CMD);
    }
}

static void oled_module_write_byte( OLED* self,				\ 
									unsigned int dat, 		\
									enum data_type_t type)
{
	int status;
	unsigned int write_buffer[1];

	if ( type == ENUM_WRITE_TYPE_CMD ) 
		self->master->set_dc_low( self );
	else 
		self->master->set_dc_high( self );
	write_buffer[0] = dat;
	write_buffer[1] = 0xFF;
	status = spi_write( self->hw.spi_dev, write_buffer, 1 );
	if ( status  )
		dev_err( &self->hw.spi_dev->dev, "%s error %d\n", __FUNCTION__, status );
}

static void oled_module_display_on( OLED* self )
{
	self->master->write_byte( self, 0x8d, ENUM_WRITE_TYPE_CMD );
	self->master->write_byte( self, 0x14, ENUM_WRITE_TYPE_CMD );
	self->master->write_byte( self, 0xaf, ENUM_WRITE_TYPE_CMD );
}

static void oled_module_display_off(OLED * self)
{
	self->master->write_byte( self, 0x8d, ENUM_WRITE_TYPE_CMD );
	self->master->write_byte( self, 0x10, ENUM_WRITE_TYPE_CMD );
	self->master->write_byte( self, 0xae, ENUM_WRITE_TYPE_CMD );
}

static void oled_module_show_char( OLED* self, int x, int y, unsigned char chr )
{
	unsigned char c = 0, i = 0;
	
    c = chr - ' ';
    if ( x > OLED_MAX_COLUMN - 1 ) {	
		x = 0; 
		y = y + 2; 
	}
    if ( OLED_SIZE == 16 ) {
        self->master->setpos( self, x, y );
        for (i = 0; i < 8; ++i) 
            self->master->write_byte( self, F8X16[c*16+i], ENUM_WRITE_TYPE_DATA );
        self->master->setpos( self, x, y + 1);
        for (i = 0; i < 8; ++i) 
            self->master->write_byte( self, F8X16[c*16+i+8], ENUM_WRITE_TYPE_DATA );
    } else {
        self->master->setpos( self, x, y+1 );
        for (i = 0; i < 6; ++i) 
            self->master->write_byte( self, F6x8[c][i], ENUM_WRITE_TYPE_DATA );

    }
}

static void oled_module_show_str( OLED* self, int x, int y, unsigned char *chr )
{
    unsigned char j = 0;
	
    while( chr[j] != '\0' ) {
        self->master->show_char( self, x, y, chr[j] );
        x += 8;
        if ( x > 120 ) {
			x = 0; 
			y += 2;	
		}
        j++;
    }
}

static void oled_module_set_pos( OLED* self, int x, int y )
{
	self->master->write_byte( self, 0xb0 + y, ENUM_WRITE_TYPE_CMD );
	self->master->write_byte( self, ( (x & 0xf0) >> 4) | 0x10, ENUM_WRITE_TYPE_CMD );
	self->master->write_byte( self, ( (x & 0x0f) | 0x01), ENUM_WRITE_TYPE_CMD );
}

module_init(oled_driver_init);
module_exit(oled_driver_exit);

MODULE_AUTHOR( DRV_AUTHOR );
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");



