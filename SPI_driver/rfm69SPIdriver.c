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
#include "/home/joao/embedded_project/raspberry_pi_SPI_driver/SPI_driver/libraries/RFM69registers.h"
#include <linux/interrupt.h>

#define DRIVER_NAME "rfm69_driver"
#define DEVICE_NAME "RFM69"
#define RF_MAJOR 177

#define PIN_NUMBER 24
#define VERSION_REGISTER 0x10
#define VERSION_DATA 0x24
#define NODE_ID 3
#define NETWORK_ID 10
#define IRQPin  29



//later put here irq pin number->after communication with RFM69 is finished
static DEFINE_MUTEX(device_lockm);
struct rfm69
{
    struct mutex          lock;
    dev_t			devt;
   // struct device         *dev;
  //  struct gpio_chip      chip;//lets hope this works on the rasp->possible huge problem
    struct spi_device     *spi; 
};
static struct class *rfm_class;
struct spi_device *spi_local;
struct rfm69 *myrf_global;

/* Define GPIOs for RX signal */
static struct gpio signals[] = {
		{ IRQPin, GPIOF_IN, "RX Signal" },	// Rx signal
};
static int rx_irqs[] = { -1 };

	/*configurations for the transceiver*/
  const u8 rf_config[] =
  {
    /* 0x01 */  REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY ,//turn on the sequencer, turn off listen, put the transceiver on standby
    /* 0x02 */  REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 , // no shaping
    /* 0x03 */  REG_BITRATEMSB, RF_BITRATEMSB_50000   , // speed of 250kbps->might be to fast testing needed
    /* 0x04 */  REG_BITRATELSB, RF_BITRATELSB_50000,
    /* 0x05 */  REG_FDEVMSB, RF_FDEVMSB_50000, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */  REG_FDEVLSB, RF_FDEVLSB_50000,	
    /* 0x07 */  REG_FRFMSB, (uint8_t) RF_FRFMSB_433    ,//put frequency of 433 MH<
    /* 0x08 */  REG_FRFMID, (uint8_t)RF_FRFMID_433 ,
    /* 0x09 */  REG_FRFLSB, (uint8_t) RF_FRFLSB_433 ,
	/* 0x0D*/   REG_LISTEN1, RF_LISTEN1_RESOL_RX_262000  |RF_LISTEN1_RESOL_IDLE_64 |RF_LISTEN1_CRITERIA_RSSIANDSYNC |RF_LISTEN1_END_01,//it goes to MODE when  PayloadReady or Timeout interrupt occurs	
    /* 0x0E*/   REG_LISTEN2, RF_LISTEN2_COEF_IDLE, // ListenCoefIdle is 1
	/* 0#define IRQPin  29x0F*/   REG_LISTEN3, RF_LISTEN3_COEFRX_VALUE, // ListenCoefRX is 0x26
	// looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
   /* 0x11 */  REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111,
   /* 0x13 */  REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 , // over current protection (default is 95mA)
    /* 0x19 */  REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 , // (BitRate < 2 * RxBw) -> need to test this configuration
    /* 0x25 */  REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 , // DIO0 is the only IRQ we're using
    /* 0x26 */  REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF , // DIO5 ClkOut disable for power saving
    /* 0x28 */  REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN , // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */  REG_RSSITHRESH, 220 , // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    /* 0x2D */  REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE , // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */  REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 ,
    /* 0x2F */  REG_SYNCVALUE1, 0x2D ,      // sync value
    /* 0x30 */  REG_SYNCVALUE2, NETWORK_ID , // NETWORK ID
   /* 0x37 */  REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_NODE ,
    /* 0x38 */  REG_PAYLOADLENGTH, 0x0A , // packet length of 10 bytes -> also needs testing
	/* 0x39 */  REG_NODEADRS, NODE_ID , 
    /* 0x3C */  REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE , // TX on FIFO not empty ->also need test
  	/* 0x3D */  REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_64BITS   | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF , // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent), AES encryption enabled
	/* 0x6F */  REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 , // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
   
 };


// // //write config function
// // //se how to implement the configurations best maybe to put it here and do it like stm
// // // after probe specific code is done but caller inside probe function
// // //spi comunication test function
// static int rfm69_spi_config(struct rfm69 *myrf)
// {
//     size_t len =1;
//     short int *i=(short int)0x10;devt
//     return spi_write(&myrf->spi,&i,len);
// }
// //spi message transmit
// int rfm69_send_message(struct rfm69 *myrf, u16 *buf, u8 len)
// {
//     int err;
//     struct spi_transfer transfer=
//     {#define IRQPin  29
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


//spi read register
u8 rfm69_read_register(struct rfm69 *myrf, u8 reg)
{
    int value=5;
    u8 buf[2]={reg,0xff};
    u8 rx_buf[2]={0};
        struct spi_transfer transfer=
    {
            .tx_buf=buf,
            .rx_buf=rx_buf,
            .len=2,
            .bits_per_word=8,
    };
        struct spi_message message;
    printk("read\n");

    if(myrf==NULL)
    {
        printk("error read register\n");
        return -ENODEV;
    }
    mutex_lock(&myrf->lock);
    spi_message_init(&message);
    spi_message_add_tail(&transfer,&message);
    value=spi_sync(myrf->spi,&message);
    if(value<0)
    {
        printk("message error\n");//see if we can change something here
    }

    //value=spi_w8r16(myrf->spi,reg);
    printk("value %d\n b1 %d b2 %d",value, rx_buf[0], rx_buf[1]);
    mutex_unlock(&myrf->lock);
    return value&0xff;
}

// //spi write register
// u8 rfm69_write_register(struct rm69 *myrf, u8 reg, u8 value)
// {
//     u16 spi_value=(((reg|0x80)<<8)|value);
//     spi_write(myrf->spi,spi_value,1);
//     return value&0xff;
// }
static int rfm69_open(struct inode *inode, struct file *f)
{
    struct rfm69 *myrf;
    int err;
    u8 version;
    struct spi_transfer transfer=
    {
            .tx_buf=rf_config,
            .len=58,
            .bits_per_word=8,
    };
    struct spi_message message;
   

    printk("open\n");
    mutex_lock(&device_lockm);
     f->private_data=myrf_global;
     nonseekable_open(inode, f);
    if(myrf_global==NULL)
    {
        printk("error read register open\n");
        return -ENODEV;
    }

    version=rfm69_read_register(myrf_global,REG_VERSION);
    if(version!=VERSION_DATA)
    {
        printk("cannot comunicate with the RF transceiver version:%d\n",version);//see if we can change something here
        mutex_unlock(&device_lockm);

         return -ENODEV;
    }    
   /* spi_message_init(&message);
    spi_message_add_tail(&transfer , &message);
    err=spi_sync(myrf->spi , &message);
    if(err<0)
    {
        printk("rf transceiver configuration error\n");//see if we can change something here
        mutex_unlock(&device_lockm);

         return err;
    }
*/
    mutex_unlock(&device_lockm);
    return err;
}

/*
        probe function it has the role of estabilishing communication between 
        our board and the peripheral, is called automaticly by the system
*/
static int rfm69_probe(struct spi_device *spi)
{
    int ret;
    unsigned long		minor;
    struct rfm69 *myrf;
    struct device *dev;
    printk( "probe\n");

    myrf_global= kzalloc(sizeof(*myrf_global),GFP_KERNEL);
    if(myrf_global==NULL)
    {
        printk("failed to to alloc memory for device\n");
    }
    myrf_global->spi=spi;
   // myrf_global->devt=&spi->dev;
    mutex_init(&myrf_global->lock);

	

	myrf_global->devt = MKDEV(RF_MAJOR, 0);
   

	dev = device_create(rfm_class, &spi->dev, myrf_global->devt,
         			   myrf_global, "rfm69_%d.%d",
				    spi->master->bus_num, spi->chip_select);
	ret = IS_ERR(dev) ? PTR_ERR(dev) : 0;

    if(ret==0)
    {
        spi_set_drvdata(spi,myrf_global);
      
    }
    else
    {
        kfree(myrf_global);
        return ret;
    }
    spi_local=spi;

    spi->mode = (SPI_MODE_0);
    spi->bits_per_word=8;
    spi->max_speed_hz = 8000000;//8Mhz
    spi->cs_gpio=24;
    ret = spi_setup(spi);
    if(ret<0)
    {
        printk("configuration");
        return ret;
    }
    else
    {
        printk("ok\n");
    }
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
    printk("remove\n");
	myrf = spi_get_drvdata(spi);
	if (!myrf)
		return -ENODEV;
    device_destroy(rfm_class, myrf->devt);
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
static irqreturn_t rx_isr(int irq, void *data)
{
 	return IRQ_HANDLED;

}
static const struct file_operations rfm_fops =
 {
	.owner =	THIS_MODULE,

	/*.write =	spidev_write,
	.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,*/
	.open =		rfm69_open,
	//.release =	spidev_release,
	//.llseek =	no_llseek,*/
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
    status = gpio_request_array(signals, ARRAY_SIZE(signals));
    if (status)
    {
        printk(KERN_ERR "RFRPI - Unable to request GPIOs for RX Signals: %d\n", status);
        free_irq(rx_irqs[0], NULL);
        class_destroy(rfm_class);
		unregister_chrdev(RF_MAJOR, rfm69_driver.driver.name);
        return status;
    }
     // Register IRQ for this GPIO
    status = gpio_to_irq(signals[0].gpio);
    if(status < 0) {
        printk(KERN_ERR "RFRPI - Unable to request IRQ: %d\n", status);
         free_irq(rx_irqs[0], NULL);
        class_destroy(rfm_class);
		unregister_chrdev(RF_MAJOR, rfm69_driver.driver.name);
        return status;
    }
    rx_irqs[0] = status;
    printk(KERN_INFO "RFRPI - Successfully requested RX IRQ # %d\n", rx_irqs[0]);
    status = request_irq(rx_irqs[0], rx_isr, IRQF_TRIGGER_RISING , "rfrpi#rx", NULL);
    if(status) {
        printk(KERN_ERR "RFRPI - Unable to request IRQ: %d\n", status);
        gpio_free_array(signals, ARRAY_SIZE(signals));
        class_destroy(rfm_class);
		unregister_chrdev(RF_MAJOR, rfm69_driver.driver.name);
        return status;
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
    struct rfm69 *myrf = spi_get_drvdata(spi_local);

    printk( "exit\n");
    // if( spi_device_rfm )
    // {
    //     spi_unregister_device( spi_device_rfm );
    // }
	spi_unregister_driver(&rfm69_driver);
    //device_destroy(rfm_class, myrf->devt);
    class_destroy(rfm_class);
    // free irqs
	free_irq(rx_irqs[0], NULL);	
	
	// unregister
	gpio_free_array(signals, ARRAY_SIZE(signals));  

	unregister_chrdev(RF_MAJOR, rfm69_driver.driver.name);
}

module_init(rfm69_init);
module_exit(rfm69_exit);

MODULE_AUTHOR("Joao Azevedo and Joao Reis");
MODULE_DESCRIPTION("SPI driver for communiction with RFM69 transceiver");
MODULE_LICENSE("GPL");


