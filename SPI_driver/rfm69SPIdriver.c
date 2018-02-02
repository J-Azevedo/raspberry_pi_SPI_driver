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

#define DRIVER_NAME "rfm69_driver"
#define DEVICE_NAME "RFM69"

#define PIN_NUMBER 24
#define VERSION_REGISTER 0x10
#define VERSION_DATA 0x24

//later put here irq pin number->after communication with RFM69 is finished

struct rfm69
{
    struct mutex          lock;
    //put here value for configurations
    //put here version register and return value
    u8              version_register;
    u8              version;
    //V probably not needed, or if needed only for the end when implementing reset pin
  //  struct gpio_chip      chip;//lets hope this works on the rasp->possible huge problem
    struct spi_device     *spi; 
};
struct rfm69_platform_data 
{
	/* number assigned to the first GPIO */
	unsigned	base;
};



//write config function
//spi comunication test function
static int rfm69_spi_config(struct rfm69 *mc)
{
    size_t len =1;
    short int *i=(short int)0x10;
    return spi_write(&mc->spi,&i,len);
}
//spi message transmit
//spi message receive

//spi read register




/*
        probe function it has the role of estabilishing communication between 
        our board and the peripheral, is called automaticly by the system
*/
static int rfm69_probe(struct spi_device *spi)
{
    int ret;
    struct rfm69 *mc;
    struct rfm69_platform_data *pdata;
    printk( "probe");
    pdata = spi->dev.platform_data;
    if(!pdata || !pdata->base)
    {
        printk(  "error incorret or missing platform data");
        return -EINVAL;
    }
    spi->mode = (SPI_MODE_0);
    spi->bits_per_word=16;
    spi->max_speed_hz = 8000000;//8Mhz
    spi->cs_gpio=24;
    ret = spi_setup(spi);
    if(ret<0)
    {
        printk("configuration");
        return ret;
    }
   /*  //see later what to put more
    mc=kzalloc(sizeof(struct rfm69),GFP_KERNEL);
    mutex_init(&mc->lock);
    dev_set_drvdata(&spi->dev,mc);
   //fix this part
    mc->version_register=VERSION_REGISTER;
    mc->version=VERSION_DATA;
    mc->spi = spi;
    mc->chip.label=DRIVER_NAME;
    //mc->chip.set=rfm69_set;->need to create set function to change pin output->do later
    mc->chip.base = pdata->base;
    mc->chip.ngpio = PIN_NUMBER;
    mc->chip.can_sleep = 1;
    mc->chip.dev = &spi->dev;
    mc->chip.owner = THIS_MODULE; */
    return ret;
}


static int rfm69_remove(struct spi_device *spi)
{
	struct rfm69 *mc;

	mc = spi_get_drvdata(spi);
	if (!mc)
		return -ENODEV;

	mutex_destroy(&mc->lock);

	return 0;
}
struct spi_board_info __initdata  spi_board_info[] = {
    {
    .modalias = DRIVER_NAME,
    .bus_num = 0,
    .chip_select = 0,
    },
};


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
    printk( "init\n");
    spi_register_board_info(spi_board_info, 1);
	return spi_register_driver(&rfm69_driver);
}

static void __exit rfm69_exit(void)
{
    printk( "exit\n");
	spi_unregister_driver(&rfm69_driver);
}

module_init(rfm69_init);
module_exit(rfm69_exit);

MODULE_AUTHOR("Joao Azevedo and Joao Reis");
MODULE_DESCRIPTION("SPI driver for communiction with RFM69 transceiver");
MODULE_LICENSE("GPL");


