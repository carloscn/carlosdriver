#ifndef INCLUDE_OLED_H_
#define INCLUDE_OLED_H_

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
#include <linux/spi/spi.h>
#include <cfg_type.h>
#include <cfg_gpio.h>
#include <mach/platform.h>
#include <mach/gpio_desc.h>
#include <mach/gpio.h>
#include <mach/soc.h>
#include <mach/slsi-spi.h>
#include <asm/div64.h>
#include <asm/io.h>
#include "oledfont.h"


#define             CMD_GET_TEMP                _IO( DS18B20_MAGIC, 0 )
#define             IO_HIGH                     1
#define             IO_LOW                      0
#define				OLED_RES_IO					PAD_GPIO_D + 0
#define				OLED_DC_IO					PAD_GPIO_B + 30
#define				OLED_CS_IO					PAD_GPIO_C + 30
#define 			OLED_SIZE 					16
#define 			OLED_L_LEVEL				0x02
#define 			OLED_X_LEVEL				0x10
#define 			OLED_MAX_COLUMN				128
#define 			OLED_MAX_ROV				64
#define				OLED_BREIGHTNESS			0xFF
#define 			OLED_X_WIDTH 				128
#define 			OLED_Y_WIDTH 				64

typedef struct oled_t    OLED;

enum data_type_t 
{
	ENUM_WRITE_TYPE_CMD = 0,
	ENUM_WRITE_TYPE_DATA,
};

struct oled_hw_t 
{
	unsigned int res_io_num;
	unsigned int dc_io_num;
	struct spi_transfer		spi_trans;
	struct spi_message		spi_msg;
	struct spi_driver		*spi_drv;
	struct spi_device		*spi_dev;
	struct spi_master		*spi_master_bus;
};

struct oled_t 
{
	OLED *self;
	struct oled_hw_t 		hw;
	/* mutex mechanism */
    struct mutex        	mutex_oled_time;
    struct spinlock         spinlock_oled_time;
	struct oled_master_t* 	master;	
	unsigned int 			majoy_id;
};

struct oled_master_t 
{
	void 	(*init)			( OLED * );
	void	(*hw_init)		( OLED * );
	void	(*reset)		( OLED * );
	void 	(*clear)		( OLED * );
	void	(*write_byte)	( OLED *, unsigned int, enum data_type_t );
	void	(*show_str) 	( OLED *, int, int, unsigned char * );
	void	(*setpos)		( OLED *, int, int );
	void	(*display_on)	( OLED * );
	void	(*display_off)	( OLED * );
	void 	(*show_char)	( OLED *, int, int, unsigned char );
	void	(*set_res_low)	( OLED * );
	void	(*set_res_high)	( OLED * );
	void	(*set_dc_low)	( OLED * );
	void	(*set_dc_high)	( OLED * );
};


OLED* 			oled_dev_new			( void );
static int  	oled_bus_spi_probe		( struct spi_device *spi );
static int  	oled_bus_spi_remove		( struct spi_device *spi );
static int 		oled_bus_spi_suspend	( struct spi_device *spi, 	\
						  					pm_message_t mesg );

static size_t 	oled_driver_write		( struct file*, 			\
										  	const char __user*, 	\
										  	size_t, loff_t* ) ;		
static size_t 	oled_driver_read		( struct file*, 			\
											const char __user*,		\ 
											size_t, loff_t* ) ;
static int		oled_driver_ioctl		( struct file*, 			\
											unsigned int,			\ 
											unsigned long );
static int 		oled_driver_open		( struct inode*, struct file* ) ;

static void		__init oled_driver_init	( void );
static void 	__exit oled_driver_exit	( void );

static void 	oled_module_init		( OLED *self );
static void 	oled_module_hw_init		( OLED *self );
static void 	oled_module_reset		( OLED *self );
static void 	oled_module_clear		( OLED *self );
static void 	oled_module_write_byte	( OLED* self, unsigned int dat, enum data_type_t type );
static void 	oled_module_display_on	( OLED* self );
static void 	oled_module_display_off	( OLED* self);
static void 	oled_module_set_pos		( OLED* self, int x, int y );
static void 	oled_module_show_str	( OLED* self, int x, int y, unsigned char *chr );
static void 	oled_module_show_char	( OLED* self, int x, int y, unsigned char chr );
static void 	oled_module_dc_set_high	( OLED *self);
static void 	oled_module_dc_set_low	( OLED *self );
static void 	oled_module_res_set_high( OLED *self );
static void 	oled_module_res_set_low	( OLED *self );




#endif

