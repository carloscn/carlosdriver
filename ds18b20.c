
/*
 * DS18B20 of Dallas driver code for Texas Instruments OMAPL138+
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

#include <cfg_type.h>
#include <cfg_gpio.h>
#include <mach/platform.h>
#include <mach/gpio_desc.h>
#include <mach/gpio.h>
#include <mach/soc.h>

#include <asm/div64.h>
#include <asm/io.h>

/**
* \brief    connector.
*
*              SP6818/
*             ---------------
*            |               |
*    sdi <-->|GPIOC14        |
*            |               |
*
*/
#define				__SAMSUNG_SP6818			0
#define				__TI_OMAPL138				1

#define             DRV_AUTHOR                  "Wei haochen <weihaochen@mltbns.com>"
#define             DRV_DESC                    "DS18B20 Dallas / OMAPL138-GPIO[0,2]"
#define             DRV_NAME                    "DS18B20"

#define             DS18B20_MAJOR               231
#define             DS18B20_SIZE                0x0010
#define             MEM_CLEAR                   0x01
#define             DS18B20_MAGIC               'q'

#define             CMD_GET_TEMP                _IO( DS18B20_MAGIC, 0 )

#define             IO_HIGH                     1
#define             IO_LOW                      0

#define             DS18B20_SDI_IO              PAD_GPIO_C + 14
#define             IO_SDI(x,y)                 gpio_direction_output( DS18B20_SDI_IO ,x ); udelay(y)

#define             KEY_IO                      PAD_GPIO_C+4


/* DS18B20 one wire family code. */
#define             DS18B20_FAMILY_CODE         0x28
/* Commands. */
#define             CONVERT_T                   0x44
#define             READ_SCRATCHPAD             0xbe
#define             WRITE_SCRATCHPAD            0x4e
#define             COPY_SCRATCHPAD             0x48
#define             RECALL_E                    0xb8
#define             READ_POWER_SUPPLY           0xb4

#define             SKIP_ROM                    0xcc
#define             CONVERT_T                   0x44
#define             READ_SCRATCHPAD             0xbe

const unsigned char  crc_table[256]={

        0,  94, 188,  226,  97,  63,  221,  131,  194,  156,  126,  32,  163,  253,  31,  65,
        157,  195,  33,  127,  252,  162,  64,  30,  95,  1,  227,  189,  62,  96,  130,  220,
        35,  125,  159,  193,  66,  28,  254,  160,  225,  191,  93,  3,  128,  222,  60,  98,
        190,  224,  2,  92,  223,  129,  99,  61,  124,  34,  192,  158,  29,  67,  161,  255,
        70,  24,  250,  164,  39,  121,  155,  197,  132,  218,  56,  102,  229,  187,  89,  7,
        219,  133, 103,  57,  186,  228,  6,  88,  25,  71,  165,  251,  120,  38,  196,  154,
        101,  59, 217,  135,  4,  90,  184,  230,  167,  249,  27,  69,  198,  152,  122,  36,
        248,  166, 68,  26,  153,  199,  37,  123,  58,  100,  134,  216,  91,  5,  231,  185,
        140,  210, 48,  110,  237,  179,  81,  15,  78,  16,  242,  172,  47,  113,  147,  205,
        17,  79,  173,  243,  112,  46,  204,  146,  211,  141,  111,  49,  178,  236,  14,  80,
        175,  241, 19,  77,  206,  144,  114,  44,  109,  51,  209,  143,  12,  82,  176,  238,
        50,  108,  142,  208,  83,  13,  239,  177,  240,  174,  76,  18,  145,  207,  45,  115,
        202,  148, 118,  40,  171,  245,  23,  73,  8,  86,  180,  234,  105,  55,  213, 139,
        87,  9,  235,  181,  54,  104,  138,  212,  149,  203,  41,  119,  244,  170,  72,  22,
        233,  183,  85,  11,  136,  214,  52,  106,  43,  117,  151,  201,  74,  20,  246,  168,
        116,  42,  200,  150,  21,  75,  169,  247,  182,  232,  10,  84,  215,  137,  107,  53
};

typedef struct ds18b20_t    DS18B20;
typedef struct sdi_master   SDI_MASTER; 

/* Declare functions: */
DS18B20*            ds18b20_dev_new();
static int          ds18b20_module_hw_read_line ( DS18B20 * );
static void         ds18b20_module_hw_set_low   ( DS18B20 * );
static void         ds18b20_module_hw_set_high  ( DS18B20 * );
static int          ds18b20_module_read_id      ( DS18B20 * , unsigned short *);
static void         ds18b20_module_config       ( DS18B20 * );
static void         ds18b20_module_convert      ( DS18B20 *);
static unsigned int ds18b20_module_check_crc    ( DS18B20 *, unsigned short );
static int          ds18b20_module_read_temp    ( DS18B20 * );
static int          ds18b20_module_read_byte    ( DS18B20 *, int * );
static int          ds18b20_module_read_bytes   ( DS18B20 *, unsigned short );
static int          ds18b20_module_write_byte   ( DS18B20 *, unsigned short );
static int          ds18b20_module_write_bytes  ( DS18B20 *, unsigned short *, unsigned short );
static int          ds18b20_module_reset        ( DS18B20 * );
static int          ds18b20_module_init         ( DS18B20 * );
static ssize_t      ds18b20_driver_write        ( struct file *, const char __user *, size_t, loff_t * );
static ssize_t      ds18b20_driver_read         ( struct file *, const char __user *, size_t, loff_t * );
static int          ds18b20_driver_ioctl        ( struct file *, unsigned int , unsigned long  );
static int          ds18b20_driver_open         ( struct inode*, struct file * );

/* Device describe */
struct ds18b20_hw_t {
    unsigned int sdi;
};


struct  ds18b20_t {
    /* Self Linked list. */
    DS18B20             *dev;
    SDI_MASTER          *master;
    /* DS18B20 hardware. */
    struct ds18b20_hw_t hw;
    /* mutex mechanism */
    struct mutex        ds_mutex;
    spinlock_t          ds_spinlock;
    
    /* copytouser or copyfromuser space. */
    int     mem[ DS18B20_SIZE ];
    /* ds18b20 data collection. */
    int     temp_value;
    char    temp_str[8];
    char    read_buffer[16];
    char    *op;
    char    id_buffer[8];
    
};

struct sdi_master {
    
    /* ds18b20 function. */
    void    (*hw_set_high)      ( DS18B20 * );
    void    (*hw_set_low)       ( DS18B20 * );
    int     (*hw_read_state)    ( DS18B20 * );
    int     (*init)             ( DS18B20 * );
    int     (*reset)            ( DS18B20 * );
    int     (*read_byte)        ( DS18B20 *, int * );
    int     (*read_bytes)       ( DS18B20 *, unsigned short );
    int     (*write_byte)       ( DS18B20 *, unsigned short );
    int     (*write_bytes)      ( DS18B20 *, unsigned short *, unsigned short );
    int     (*read_temp)        ( DS18B20 * );
    int     (*config)           ( DS18B20 * );
    int     (*read_id)          ( DS18B20 *, unsigned short *);
    unsigned int (*check_crc)   ( DS18B20 *, unsigned short);
    void    (*convert)          ( DS18B20 * );

};

struct gpio_irq_desc {

    int irq;
    unsigned long flags;
    char *name;

} press_dev_desc = {

        KEY_IO,
        IRQ_TYPE_EDGE_FALLING,
        "sw6_push_button"

};

DS18B20 *ds18b20;

static int  ds18b20_major   =   DS18B20_MAJOR;

DS18B20 *ds18b20_dev_new()
{
    DS18B20 *dev_p = (DS18B20*)kcalloc(1, sizeof(DS18B20), GFP_ATOMIC);
    
    dev_p->master = (SDI_MASTER*)kcalloc(1, sizeof(SDI_MASTER), GFP_ATOMIC);
    dev_p->hw.sdi                   =   DS18B20_SDI_IO;
    dev_p->master->init             =   &ds18b20_module_init;
    dev_p->master->reset            =   &ds18b20_module_reset;
    dev_p->master->read_byte        =   &ds18b20_module_read_byte;
    dev_p->master->read_bytes       =   &ds18b20_module_read_bytes;
    dev_p->master->write_byte       =   &ds18b20_module_write_byte;
    dev_p->master->write_bytes      =   &ds18b20_module_write_bytes;
    dev_p->master->read_temp        =   &ds18b20_module_read_temp;
    dev_p->master->hw_read_state    =   &ds18b20_module_hw_read_line;
    dev_p->master->hw_set_high      =   &ds18b20_module_hw_set_high;
    dev_p->master->hw_set_low       =   &ds18b20_module_hw_set_low;
    dev_p->master->read_id          =   &ds18b20_module_read_id;
    dev_p->master->check_crc        =   &ds18b20_module_check_crc;
    dev_p->master->config           =   &ds18b20_module_config;
    dev_p->master->convert          =   &ds18b20_module_convert;
    
    dev_p->master->init( dev_p );
	
    return dev_p;
}


static void ds18b20_module_hw_set_high( DS18B20 *dev)
{
#if __TI_OMAPL138
    gpio_direction_output( dev->hw.sdi , IO_HIGH );
#elif __SAMSUNG_SP6818
	nxp_soc_gpio_set_out_value( dev->hw.sdi , IO_HIGH);
#endif
}

static void ds18b20_module_hw_set_low( DS18B20 *dev )
{
#if __TI_OMAPL138
	gpio_direction_output( dev->hw.sdi , IO_LOW );
#elif __SAMSUNG_SP6818
	nxp_soc_gpio_set_out_value( dev->hw.sdi , IO_LOW);
#endif
	
}

static int ds18b20_module_hw_read_line( DS18B20 *dev )
{
    int ret;
#if __TI_OMAPL138
    gpio_direction_input( dev->hw.sdi );

    if ( gpio_get_value( dev->hw.sdi ) != 0 ) 
        ret = 1;
    else
        ret = 0;
#elif __SAMSUNG_SP6818
	nxp_soc_gpio_set_io_dir(dev->hw.sdi, 0);

	if ( nxp_soc_gpio_get_in_value( dev->hw.sdi ) != 0 ) 
        ret = 1;
    else
        ret = 0;
#endif
    return ret;
}

static int ds18b20_module_init( DS18B20 *dev )
{
    int ret;
    
    dev->master->reset(dev);
    dev->master->write_byte(dev, SKIP_ROM);
    dev->master->write_byte(dev, CONVERT_T);

    printk( DRV_NAME "\tWrite init infomation\n" );

    return ret;
}

static void ds18b20_module_config( DS18B20 *dev ) 
{
    dev->master->reset(dev);
    dev->master->write_byte( dev, 0xcc);  
    dev->master->write_byte( dev, 0x4e);  
    dev->master->write_byte( dev, 0x19);  
    dev->master->write_byte( dev, 0x1a);  
    dev->master->write_byte( dev, 0x7f);  
    dev->master->reset( dev );
    dev->master->write_byte( dev, 0xcc);  
    dev->master->write_byte( dev, 0x48); 
    dev->master->reset( dev );
    dev->master->write_byte( dev, 0xcc);  
    dev->master->write_byte( dev, 0xb8); 
    
}

static int ds18b20_module_reset( DS18B20 *dev )
{
    int ret;
    unsigned long flag;
    
    spin_lock_irqsave(&dev->ds_spinlock, flag);

    dev->master->hw_set_low( dev );
    udelay(490);
    dev->master->hw_set_high( dev );
    udelay(100);
    udelay(480);
    dev->master->hw_set_high( dev );

    spin_unlock_irqrestore(&dev->ds_spinlock, flag);
    
    return -1;
        
}

static int ds18b20_module_write_bytes( DS18B20 *dev, unsigned short *words, unsigned short length  )
{
    int ret,i,j;
    unsigned long flag;
    unsigned short data;
    
    spin_lock_irqsave(&dev->ds_spinlock, flag);
    for (j = 0; j < length; j++) {
        data = *(words + j);
        dev->master->write_byte(dev, data);
    }
    spin_unlock_irqrestore(&dev->ds_spinlock, flag);
    
    return ret;

}

static int ds18b20_module_write_byte( DS18B20 *dev, unsigned short byte )
{
    int ret,i;
    unsigned long flag;
    unsigned short data;
    
    data = byte;
    spin_lock_irqsave(&dev->ds_spinlock, flag);

    for (i = 0; i < 8; i++) {
        dev->master->hw_set_low( dev );
        udelay(2);
        if( data & 0x01 ) 
            dev->master->hw_set_high( dev );
        else
            dev->master->hw_set_low( dev );
        udelay(45);
        dev->master->hw_set_high( dev );
        data >>= 1;
    }
    spin_unlock_irqrestore(&dev->ds_spinlock, flag);
    
    return ret;

}

static int ds18b20_module_read_bytes( DS18B20 *dev, unsigned short length )
{
    int ret,i;
    unsigned short read_value;

    for( i = 0 ; i < length ; i ++) {
        dev->master->read_byte( dev,(dev->op) );
        dev->op ++;
    }
    ret = -1;
    return ret;
        
}
/*
    no effect.
*/
static void ds18b20_module_convert( DS18B20 *dev )
{
    char head[2];
    char tail[2];
    unsigned long long temp_head;
    unsigned long long temp_tail;
    unsigned long long cal;
    cal = 0;
    temp_head = dev->temp_value;
    cal = do_div(temp_head, 1000) + 0x30;
    head[0] = cal;
    temp_head = dev->temp_value;
    cal = do_div(temp_head, 100) + 0x30;
    head[1] = cal;
    temp_tail = dev->temp_value;
    cal = do_div( temp_tail, 10 ) + 0x30;
    tail[0] = cal;
    temp_tail = dev->temp_value;
    cal = do_div( temp_tail, 1 )  + 0x30;
    tail[1] = cal;
    
    dev->temp_str[0] = (unsigned long long )head[1];
    dev->temp_str[1] = (unsigned long long )head[0];
    dev->temp_str[2] = (unsigned long long )'.';
    dev->temp_str[3] = (unsigned long long )tail[1];
    dev->temp_str[4] = (unsigned long long )tail[0];


}
static int ds18b20_module_read_byte( DS18B20 *dev, int *byte )
{
    int ret,i;
    int read_value;
    unsigned long flag;
    
    read_value = 0;
    spin_lock_irqsave(&dev->ds_spinlock, flag);
    
    for (i = 0; i < 8; i ++) {
        
        dev->master->hw_set_low( dev);
        udelay(2);
        read_value >>= 1;

        dev->master->hw_set_high( dev );
        udelay(4);

        if( dev->master->hw_read_state( dev )  == 1 )
            read_value |= 0x80;
        udelay(65);
    }

    *(byte) = read_value;
    spin_unlock_irqrestore(&dev->ds_spinlock, flag);
    ret = -1;
    
    return ret;
}

static unsigned int ds18b20_module_check_crc( DS18B20 *dev, unsigned short checksum )
{
    unsigned int i,crc_data = 0;
    
    for( i = 0; i < checksum; i++ )
        crc_data    =   crc_table[ crc_data^( *(dev->read_buffer + i) ) ];
    return crc_data;
}

static int ds18b20_module_read_id( DS18B20 *dev, unsigned short *id_val )
{
    dev->master->reset( dev );
    dev->master->write_byte( dev, SKIP_ROM);
    dev->master->read_bytes( dev, 8 );
}

static int ds18b20_module_read_temp( DS18B20 *dev )
{
    int temp;
    int write_rom[2];
    int read_rom[2];
    
    temp = 0;
    printk(DRV_NAME "\tmutex lock ...\n");
    mutex_lock(&dev->ds_mutex);
    dev->op = dev->id_buffer;
    printk(DRV_NAME "\tread id ...\n");
    dev->master->read_id( dev, dev->id_buffer );
    printk(DRV_NAME "\tid:%s \n",dev->id_buffer);
    printk(DRV_NAME "\tconfiguration ...\n");
    dev->master->config(dev);
    dev->master->reset(dev);
    printk(DRV_NAME "\twrite ...\n");
    dev->master->write_byte( dev, SKIP_ROM );
    dev->master->write_byte( dev, CONVERT_T );
    dev->master->reset( dev );
    dev->master->write_byte( dev, SKIP_ROM );
    dev->master->write_byte( dev, READ_SCRATCHPAD );
    printk(DRV_NAME "\twrite ...ok....\n");
    dev->op = dev->read_buffer;
    printk(DRV_NAME "\tread ...\n");
    dev->master->read_bytes(  dev , 9 ) ;
    printk(DRV_NAME "\tread ...ok....\n");
    printk(DRV_NAME "\tcheck crc ....\n");
    if ( dev->master->check_crc( dev, 9) == 0 ) {
        temp = dev->read_buffer[1] * 0x100 + dev->read_buffer[0];
        temp *= 100;
        do_div(temp, 16);
        udelay(10);
        printk(DRV_NAME "\tcheck crc ...ok...\n");
    }else {
        printk(DRV_NAME "\tcheck crc ...failed...\n");
    }
    dev->temp_value = temp;
    //dev->convert( dev );                                           
    mutex_unlock(&dev->ds_mutex);
    printk(DRV_NAME "\tmutex unlock ...\n");
    return (int)temp;    
}

static const short  ds18b20_gpios[] = {
        DS18B20_SDI_IO,
        KEY_IO,
};


static int key_count = 0;
static irqreturn_t  ds18b20_press_intHandle( int irq, void *dev_id )
{
    int temp_value;
    unsigned char *temp_str;
    unsigned char *res;

    printk( DRV_NAME "\tPress trigger,getting the temp value!\n" );
    temp_value = ds18b20->master->read_temp(ds18b20);
    ds18b20->mem[0] = temp_value;
    printk(DRV_NAME "\tThe temp is : %d \n", ds18b20->temp_value);  
    return  IRQ_RETVAL( IRQ_HANDLED );
}


static int  ds18b20_driver_open( struct inode *inodes, struct file *filp )
{
    int ret;
    printk(DRV_NAME "\tDS18B20 openned.\n");
    return 0;
}


static int  ds18b20_driver_ioctl(struct file  *file, unsigned int cmd, unsigned long arg )
{

    printk(DRV_NAME "\tRecv cmd: %u\n", cmd);
    printk(DRV_NAME "\tRecv arg: %lu\n", arg);

    return  0;
}


static ssize_t  ds18b20_driver_read( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos )
{
    unsigned long   p       =   *f_pos;
    unsigned int    count   =   size;
    int             ret     =   0;
    int             temp;

    temp = ds18b20->master->read_temp(ds18b20);
    //read five temp value;
    ds18b20->mem[0] = temp;
    if ( p >= DS18B20_SIZE )
        return 0;
    if ( count > DS18B20_SIZE - p )
        count = DS18B20_SIZE - p;
    if ( copy_to_user( buffer, ds18b20->mem + p, count) ) {
        ret =   -EFAULT;
    }else {
        *f_pos += count;
        ret =   count;
        printk( DRV_NAME "\tread %u bytes from %lu\n", count, p );
        printk( DRV_NAME "\tread: %d \n", *buffer );
    }
    return ret;
}

static ssize_t  ds18b20_driver_write( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos )
{
    unsigned long   p       =   *f_pos;
    unsigned int    count   =   size;
    int             ret     =   0;

    if ( p >= DS18B20_SIZE )
        return 0;
    if ( count > DS18B20_SIZE - p )
        count = DS18B20_SIZE - p;

    memset( ds18b20->mem,0, DS18B20_SIZE );

    if ( copy_from_user( ds18b20->mem + p, buffer, count) ) {
        ret =   -EFAULT;
    }else {
        *f_pos += count;
        ret =   count;
        printk( DRV_NAME "\twrite %u bytes from %lu\n", count, p );
        printk( DRV_NAME "\tRecv: %s \n", ds18b20->mem + p );
        printk( DRV_NAME "\tSet freq is: %d \n", simple_strtol(ds18b20->mem + p,"str",0) );
        //ad9833->set_wave_freq(ds18b20, simple_strtol(ds18b20->mem + p,"str",0) );
    }
    return ret;
}


static struct file_operations ds18b20_fops = {

        .owner              =   THIS_MODULE,
        .open               =   ds18b20_driver_open,
        .read               =   ds18b20_driver_read,
        .write              =   ds18b20_driver_write,
        .unlocked_ioctl     =   ds18b20_driver_ioctl,
};

static struct miscdevice ds18b20_miscdev  = {

        .name               =   DRV_NAME,
        .fops               =   &ds18b20_fops,
        .nodename           =   DRV_NAME,
};

static int __init ds18b20_driver_init( void )
{
    int     i,ret;
    int     index_minor = 0;
    int     mk_major;
    int     temp_value;
    /*
     * DS18B20 init gpios.
     * */
    printk( DRV_NAME "\tInititial GPIO\n" );

    for ( i = 0; i < ARRAY_SIZE( ds18b20_gpios ); i ++ ) {
        ret =   gpio_request( ds18b20_gpios[i], "DS18B20 GPIO" );
        if( ret ) {
            printk( DRV_NAME "\trequest gpio %d for DS18B20 failed, ret = %d\n", ds18b20_gpios[i],ret);
            return ret;
        }else {
            printk( DRV_NAME "\trequest gpio %d for DS18B20 set succussful, ret = %d\n", ds18b20_gpios[i],ret);
        }

    }
#if __TI_OMAPL138
    gpio_direction_output( DS18B20_SDI_IO, 1 );
    gpio_set_value( DS18B20_SDI_IO, 1 );
    gpio_direction_output( KEY_IO, 0 );

#elif __SAMSUNG_SP6818
	printk( DRV_NAME "\tnxp gpio set..." );
	nxp_soc_gpio_set_io_func( DS18B20_SDI_IO, 0);
	nxp_soc_gpio_set_io_pull_enb(DS18B20_SDI_IO, 0);
	nxp_soc_gpio_set_int_enable(DS18B20_SDI_IO, 0);
	nxp_soc_gpio_set_io_dir( DS18B20_SDI_IO , 1);
	nxp_soc_gpio_set_out_value( DS18B20_SDI_IO , IO_HIGH);
	nxp_soc_gpio_set_io_drv( DS18B20_SDI_IO, 0);

#endif
    /*
     * interrupt apply
     * */
    press_dev_desc.irq =  gpio_to_irq( KEY_IO );
    ret =   request_irq( press_dev_desc.irq , &ds18b20_press_intHandle, press_dev_desc.flags, press_dev_desc.name, (void*)0 );
    if( ret ) {
        printk( DRV_NAME "\terror %d: IRQ = %d number failed!\n", ret, gpio_to_irq(KEY_IO) );
        ret = -EBUSY;
        return ret;
    }
        /*
     * DS18B20 new device
     * */
    printk( DRV_NAME "\tApply mem for DS18B20.\n" );
    ds18b20 = ds18b20_dev_new();
    if( !ds18b20 ) {
        ret = -ENOMEM;
        printk(DRV_NAME "\tDS18B20 new device failed!\n" );
        kfree( ds18b20 );
    }
    
    printk( DRV_NAME "\tiqr apply ok!!\n" );
    
    ret = misc_register( &ds18b20_miscdev );
    printk( DRV_NAME "\tgenerated a driver nod...\n" );

    spin_lock_init(&ds18b20->ds_spinlock);
    mutex_init(&ds18b20->ds_mutex);
    
    temp_value = ds18b20->master->read_temp(ds18b20);
    printk(DRV_NAME "\tThe temp is : %d \n", ds18b20->temp_value);  
	for ( i = 0; i < 2500; i ++ ) {
		
		if( i%2 == 0 ) { 
			gpio_set_value( DS18B20_SDI_IO, 1 );
		}else{
			gpio_set_value( DS18B20_SDI_IO, 0 );
		}
	}
    return 0;


}


static void __exit ds18b20_driver_exit( void )
{
    int i;
    // release gpio source.
    kfree(ds18b20);
    
    for( i = 0; i < ARRAY_SIZE(ds18b20_gpios); i++) {
        gpio_free( ds18b20_gpios[i] );
    }
    // remove driver nod.
    misc_deregister( &ds18b20_miscdev );
    // release irq
    free_irq( press_dev_desc.irq, (void*)0 );

}


module_param( ds18b20_major, int, S_IRUGO );
module_init( ds18b20_driver_init );
module_exit( ds18b20_driver_exit );


MODULE_AUTHOR( DRV_AUTHOR );
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");







