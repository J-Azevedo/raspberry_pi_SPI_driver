/*
*	Raspberry Pi SPI driver
*	implemented to interact with RFM69 transceiver from Adafruit
*
*/


#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <linux/slab.h>
#include <linux/kern_levels.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#define DRIVER_NAME "rfm69_driver"
#define DEVICE_NAME "RFM69"
#define RF_MAJOR 177

#define PIN_NUMBER 24
#define VERSION_REGISTER 0x10
#define VERSION_DATA 0x24
static struct spi_device *spi_device_rfm;
//later put here irq pin number->after communication with RFM69 is finished

struct rfm69
{
    struct mutex          lock;
    struct device         *dev;
  //  struct gpio_chip      chip;//lets hope this works on the rasp->possible huge problem
    struct spi_device     *spi; 
};
static struct class *rfm_class;



// //write config function
// //se how to implement the configurations best maybe to put it here and do it like stm
// // after probe specific code is done but caller inside probe function
// //spi comunication test function
// static int rfm69_spi_config(struct rfm69 *myrf)
// {
//     size_t len =1;
//     short int *i=(short int)0x10;
//     return spi_write(&myrf->spi,&i,len);
// }
// //spi message transmit
// int rfm69_send_message(struct rfm69 *myrf, u16 *buf, u8 len)
// {
//     int err;
//     struct spi_transfer transfer=
//     {
//             .tx_buf=buf,
//             .len=len,
//             .bits_per_word=16,
//     };
//     struct spi_message message;
//     mutex_lock(myrf->lock);
//     spi_message_init(&message);
//     spi_message_add_tail(&transfer,&message);
//     err=spi_sync(myrf->spi,&messager);
//     if(err<0)
//     {
//         printk("message error\n");//see if we can change something here
//     }
//     //add here write to reg to change mode
//     mutex_unlock(myrf->lock);
    
//     return err;
// }
// //spi read message
// int rfm69_read_message(struct rfm69 *myrf, u16 *buf, u8 len)
// {
//     int err;
    
//     struct spi_transfer transfer=
//     {
//             .tx_buf=buf,
//             .len=len,
//             .bits_per_word=16,
//     };
//         struct spi_message message;
//     mutex_lock(myrf->lock);
//     spi_message_init(&message);
//     spi_message_add_tail(&transfer,&message);
//     err=spi_sync(myrf->spi,&messager);
//     if(err<0)
//     {
//         printk("message error\n");//see if we can change something here
//     }
//     //add here write to reg to change mode
//     mutex_unlock(myrf->lock);
    
//     return err;



// }


// //spi read register
// u8 rfm69_read_register(struct rm69 *myrf, u8 reg)
// {
//     int value;
//     value=spi_w8r16(myrf->spi,reg);
//     return value&0xff
// }

// //spi write register
// u8 rfm69_write_register(struct rm69 *myrf, u8 reg, u8 value)
// {
//     u16 spi_value=((reg|0x80)<<8)|value
//     spi_write(myrf->spi,spi_value,1);
//     return value&0xff
// }

/*
        probe function it has the role of estabilishing communication between 
        our board and the peripheral, is called automaticly by the system
*/
static int rfm69_probe(struct spi_device *spi)
{
    int ret;
    struct rfm69 *myrf;
    printk( "probe\n");

    myrf= kzalloc(sizeof(*myrf),GFP_KERNEL);
    if(myrf==NULL)
    {
        printk("failed to to alloc memory for device\n");
    }
    myrf->spi=spi;
    myrf->dev=&spi->dev;
    mutex_init(&myrf->lock);
    spi_set_drvdata(spi,myrf);
    // pdata = spi->dev.platform_data;
    // if(!pdata || !pdata->base)
    // {
    //     printk(  "error incorret or missing platform data");
    //     return -EINVAL;
    // }
    // spi->mode = (SPI_MODE_0);
    // spi->bits_per_word=16;
    // spi->max_speed_hz = 8000000;//8Mhz
    // spi->cs_gpio=24;
    // ret = spi_setup(spi);
    // if(ret<0)
    // {
    //     printk("configuration");
    //     return ret;
    // }
   /*  //see later what to put more
    myrf=kzalloc(sizeof(struct rfm69),GFP_KERNEL);
    mutex_init(&myrf->lock);
    dev_set_drvdata(&spi->dev,myrf);
   //fix this part
    myrf->version_register=VERSION_REGISTER;
    myrf->version=VERSION_DATA;
    myrf->spi = spi;
    myrf->chip.label=DRIVER_NAME;
    //myrf->chip.set=rfm69_set;->need to create set function to change pin output->do later
    myrf->chip.base = pdata->base;
    myrf->chip.ngpio = PIN_NUMBER;
    myrf->chip.can_sleep = 1;
    myrf->chip.dev = &spi->dev;
    myrf->chip.owner = THIS_MODULE; */
    //here we have to add a configuration function
    return ret;
}


static int rfm69_remove(struct spi_device *spi)
{
	struct rfm69 *myrf;

	myrf = spi_get_drvdata(spi);
	if (!myrf)
		return -ENODEV;
    kfree(myrf);
	mutex_destroy(&myrf->lock);

	return 0;
}
/*struct spi_board_info __initdata  spi_board_info[] = {
    {
    .modalias = DRIVER_NAME,
    .bus_num = 0,
    .chip_select = 0,
    },
};
*/

/*
static const struct of_device_id rfm69_id_table[] = {
	{.name=DRIVER_NAME, .compatible="brcm,bcm2835-spi",},
    {},
};
MODULE_DEVICE_TABLE(of,rfm69_id_table);*/
/*static const struct spi_device_id rfm69_of_id_table[] = {
	{DRIVER_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(spi,rfm69_id_table);*/
static const struct file_operations rfm_fops = {
	.owner =	THIS_MODULE,

	/*.write =	spidev_write,
	.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,*/
};

static struct spi_driver rfm69_driver =
{
    .driver=
    {
        .name = DRIVER_NAME,
       .bus  = &spi_bus_type, 
       .owner = THIS_MODULE,
     //  .of_match_table=rfm69_id_table,
    },
    .probe    = rfm69_probe,
    .remove   = rfm69_remove,
    //.id_table = rfm69_id_table,
};

static int __init rfm69_init(void)
{
  int status;

    // struct spi_board_info spi_rfm_info =
    // {
    //     .modalias="spidev0.2",
    //     .max_speed_hz=8000000,
    //     .bus_num=0,
    //  //   .chip_select=0,->see this one later has i think i will get problems
    //     .mode=SPI_MODE_0,
    //     //irq->later
    // };
    printk( "init\n");
    status=register_chrdev(RF_MAJOR,"spi",&rfm_fops);
    if(status<0)
    {
        printk("failed to register device\n");
        return status;
    }
    rfm_class=class_create(THIS_MODULE,"RFM69");
	if (IS_ERR(rfm_class))
    {
        printk("failed to allocate class\n");
		unregister_chrdev(RF_MAJOR, rfm69_driver.driver.name);
		return PTR_ERR(rfm_class);
	}    
    status=spi_register_driver(&rfm69_driver);
    if(status<0)
    {
        printk("failed to register driver\n");
        class_destroy(rfm_class);
		unregister_chrdev(RF_MAJOR, rfm69_driver.driver.name);
        return status;
    }

	return status;
}

static void __exit rfm69_exit(void)
{
    printk( "exit\n");
    // if( spi_device_rfm )
    // {
    //     spi_unregister_device( spi_device_rfm );
    // }
	spi_unregister_driver(&rfm69_driver);
    class_destroy(rfm_class);
	unregister_chrdev(RF_MAJOR, rfm69_driver.driver.name);
}

module_init(rfm69_init);
module_exit(rfm69_exit);

MODULE_AUTHOR("Joao Azevedo and Joao Reis");
MODULE_DESCRIPTION("SPI driver for communiction with RFM69 transceiver");
MODULE_LICENSE("GPL");


