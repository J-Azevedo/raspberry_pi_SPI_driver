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
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include "/home/joao/embedded_project/raspberry_pi_SPI_driver/SPI_driver/libraries/RFM69registers.h"



#define DRIVER_NAME "rfm69_driver"
#define DEVICE_NAME "RFM69"
#define RF_MAJOR 177

#define PIN_NUMBER 24
#define VERSION_REGISTER 0x10
#define VERSION_DATA 0x24
#define NODE_ID 3
#define NETWORK_ID 10
#define IRQPin  25

#define TX_MODE RF_OPMODE_SEQUENCER_ON|RF_OPMODE_TRANSMITTER|RF_OPMODE_LISTEN_OFF



struct rfm69
{
    struct mutex          lock;
    dev_t			      devt;
    struct spi_device     *spi; 
    pid_t                  PID;
};


static struct receivedMessages
{
    u8 message[0xA];
    struct receivedMessages *next;

}*headList;

static struct receivedMessages *tailList;

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
    /* 0x01 */  REG_OPMODE|0x80, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_ON | RF_OPMODE_STANDBY,//turn on the sequencer, turn on listen mode
    /* 0x02 */  REG_DATAMODUL|0x80, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 , // no shaping
    /* 0x03 */  REG_BITRATEMSB|0x80, RF_BITRATEMSB_50000   , // speed of 250kbps->might be to fast testing needed
    /* 0x04 */  REG_BITRATELSB|0x80, RF_BITRATELSB_50000,
    /* 0x05 */  REG_FDEVMSB|0x80, RF_FDEVMSB_50000, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */  REG_FDEVLSB|0x80, RF_FDEVLSB_50000,	
    /* 0x07 */  REG_FRFMSB|0x80, (uint8_t) RF_FRFMSB_433    ,//put frequency of 433 MH<
    /* 0x08 */  REG_FRFMID|0x80, (uint8_t)RF_FRFMID_433 ,
    /* 0x09 */  REG_FRFLSB|0x80, (uint8_t) RF_FRFLSB_433 ,
	/* 0x0D*/   REG_LISTEN1|0x80, RF_LISTEN1_RESOL_RX_262000  |RF_LISTEN1_RESOL_IDLE_64 |RF_LISTEN1_CRITERIA_RSSIANDSYNC |RF_LISTEN1_END_01,//it goes to MODE when  PayloadReady or Timeout interrupt occurs	
    /* 0x0E*/   REG_LISTEN2|0x80, 0x01, // ListenCoefIdle is 1
	/* 0x0F*/   REG_LISTEN3|0x80, 0xff, // ListenCoefRX is 0x26
	// looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
   /* 0x11 */  REG_PALEVEL|0x80, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111,
   /* 0x13 */  REG_OCP|0x80, RF_OCP_ON | RF_OCP_TRIM_95 , // over current protection (default is 95mA)
    /* 0x19 */  REG_RXBW|0x80, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 , // (BitRate < 2 * RxBw) -> need to test this configuration
    /* 0x25 */  REG_DIOMAPPING1|0x80, RF_DIOMAPPING1_DIO0_01 , // DIO0 is the only IRQ we're using
    /* 0x26 */  REG_DIOMAPPING2|0x80, RF_DIOMAPPING2_CLKOUT_OFF , // DIO5 ClkOut disable for power saving
    /* 0x28 */  REG_IRQFLAGS2|0x80, RF_IRQFLAGS2_FIFOOVERRUN , // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */  REG_RSSITHRESH|0x80, 220 , // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    /* 0x2D */  REG_PREAMBLELSB|0x80, RF_PREAMBLESIZE_LSB_VALUE , // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */  REG_SYNCCONFIG|0x80, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 ,
    /* 0x2F */  REG_SYNCVALUE1|0x80, 0x2D ,      // sync value
    /* 0x30 */  REG_SYNCVALUE2|0x80, NETWORK_ID , // NETWORK ID
    /* 0x37 */  REG_PACKETCONFIG1|0x80, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_NODE ,
    /* 0x38 */  REG_PAYLOADLENGTH|0x80, 0x0A , // packet length of 10 bytes -> also needs testing
	/* 0x39 */  REG_NODEADRS|0x80, NODE_ID , 
    /* 0x3C */  REG_FIFOTHRESH|0x80, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE , // TX on FIFO not empty ->also need test
  	/* 0x3D */  REG_PACKETCONFIG2|0x80, RF_PACKET2_RXRESTARTDELAY_64BITS   | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF , // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent), AES encryption enabled
	/* 0x6F */  REG_TESTDAGC|0x80, RF_DAGC_IMPROVED_LOWBETA0 , // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
   
 };



//spi read register
static u8 rfm69_read_register(struct rfm69 *myrf, u8 reg)
{
    int value=5;
    u8 tx_buf[2]={(reg&0x7f),0xff};
    u8 rx_buf[2]={0};
        struct spi_transfer transfer=
    {
            .tx_buf=tx_buf,
            .rx_buf=rx_buf,
            .len=2,
            .bits_per_word=8,
    };
    struct spi_message message;
    printk("read register\n");

    if(myrf==NULL)
    {
        printk("failed to access spi device\n");
        return -ENODEV;
    }
    spi_message_init(&message);
    spi_message_add_tail(&transfer,&message);
    value=spi_sync(myrf->spi,&message);
    if(value<0)
    {
        printk("read register failed\n");//see if we can change something here
        return value;
    }
    printk("transmission return value %d\nmosi b1 %x b2 %x\nmiso b1 %x b2 %x\n",value,tx_buf[0],tx_buf[1], rx_buf[0], rx_buf[1]);

    return rx_buf[1];
}

//spi write register
static u8 rfm69_write_register(struct rfm69 *myrf, u8 reg, u8 value)
{
    int err;
    u8 tx_buf[2]={(reg|0x80),value};
    u8 rx_buf[2]={0};
        struct spi_transfer transfer=
    {
            .tx_buf=tx_buf,
            .rx_buf=rx_buf,
            .len=2,
            .bits_per_word=8,
    };
    struct spi_message message;
    printk("write register\n");

    if(myrf==NULL)
    {
        printk("failed to access spi device\n");
        return -ENODEV;
    }

    spi_message_init(&message);
    spi_message_add_tail(&transfer,&message);
    err=spi_sync(myrf->spi,&message);
    if(err<0)
    {
        printk("write register failed\n");//see if we can change something here

        return err;
    }
    
    printk("transmission return value %d\nmosi b1 %x b2 %x\nmiso b1 %x b2 %x\n",value,tx_buf[0],tx_buf[1], rx_buf[0], rx_buf[1]);

    return 0;
}
static int rfm69_write_FIFO_message(struct rfm69 *myrf, char *buf, int len)
{
        int err;
    struct spi_transfer transfer=
    {
            .tx_buf=buf,
            .len=len,
            .bits_per_word=8,
    };
    struct spi_message message;
   
    spi_message_init(&message);
    spi_message_add_tail(&transfer , &message);
    err=spi_sync(myrf->spi , &message);
    if(err<0)
    {
        printk("message was not transmited\n");
        

    }
    return err;


}
static ssize_t rfm69_write(struct file *filp, const char __user *buf,
		                        size_t count, loff_t *f_pos)
{
    ssize_t status=0;
    unsigned long missing;
    char *buffer;
    int err;
    
    mutex_lock(&myrf_global);
    
    buffer= kmalloc(count,GFP_KERNEL);

    missing = copy_from_user(buffer, buf, count);
    if (missing == 0)
    {
        status = rfm69_write_FIFO_message(myrf_global, buffer,count);
        kfree(buffer);
    }
	else
    {
        status = -EFAULT;

    }
    rfm69_write_register(myrf_global,REG_OPMODE,TX_MODE);
    udelay(5);
    err=(rfm69_read_register(myrf_global,REG_IRQFLAGS1)&RF_IRQFLAGS1_MODEREADY);
    if(err==0)
    {
        printk("transceiver did not transmit message\n");
        status=-ECOMM;
    }
    mutex_unlock(&myrf_global);


    return status;
}
static ssize_t 
rfm69_read(struct file *filp, const char __user *buf,
		                        size_t count, loff_t *f_pos)
{
    struct receivedMessages *temp;
    unsigned long missing;
    ssize_t err=0;
    mutex_lock(&myrf_global);

    if(headList==NULL)
    {
        printk("There is no message to read\n");
        return -ENOMSG;
    }
    missing =  copy_to_user(buf,headList->message, 0xA);
       
    if (missing == 0)
    {
        err=-1;
    }
    temp=headList->next;
    kfree(headList);
    headList=temp;
    mutex_unlock(&myrf_global);
    return err;


}



static int rfm69_config(struct rfm69 *myrf)
{
    int err;
    struct spi_transfer transfer=
    {
            .tx_buf=rf_config,
            .len=58,
            .bits_per_word=8,
    };
    struct spi_message message;
   
    spi_message_init(&message);
    spi_message_add_tail(&transfer , &message);
    err=spi_sync(myrf->spi , &message);
    if(err<0)
    {
        printk("rf transceiver configuration error\n");//see if we can change something here
    

    }
    return err;

}
static int rfm69_open(struct inode *inode, struct file *f)
{
    struct rfm69 *myrf;
    int err;
    u8 version;

    f->private_data=myrf_global;
    printk("open\n");
    mutex_lock(&myrf_global->lock);
    //need to find a way to get PID from daemon process, it can even be provided
    //by reference here


     nonseekable_open(inode, f);
    if(myrf_global==NULL)
    {
        printk("failed to access spi device\n");
        mutex_unlock(&myrf_global->lock);
        return -ENODEV;
    }

    version=rfm69_read_register(myrf_global,REG_VERSION);
    if(version!=VERSION_DATA)
    {
        printk("cannot comunicate with the RF transceiver\n");//see if we can change something here
        mutex_unlock(&myrf_global->lock);

         return -ENODEV;
    }
    err=rfm69_config(myrf_global);
    printk("The process id is %d\n", (int) task_pid_nr(current));

  


    mutex_unlock(&myrf_global->lock);
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
    int i=0, k=0;
    printk( "probe\n");

    myrf_global= kzalloc(sizeof(*myrf_global),GFP_KERNEL);
    if(myrf_global==NULL)
    {
        printk("failed to to alloc memory for device\n");
        return -ENOMEM;
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
    i=spi->master->num_chipselect;
    k=(*spi->master->cs_gpios);
    spi->cs_gpio=8;
    ret = spi_setup(spi);

    if(ret<0)
    {
        printk("failed to configure spi device\n");
        return ret;
    }
    else
    {
        printk("spi device correclty configured\n");
        printk("chip select %d\n",i);
        printk("chip select gpio %d\n",k);

    }

    
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



static irqreturn_t rx_isr(int irq, void *data)
{
    struct receivedMessages *newMessage;
    int value=5;
    u8 tx_buf[0xA]={0x0,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    siginfo_t info;
    struct spi_transfer transfer=
    {
            .tx_buf=tx_buf,
            .len=0xA,
            .bits_per_word=8,
    };
    struct spi_message message;
    printk("Message received by the transceiver\n");
    newMessage=kzalloc(sizeof(*newMessage),GFP_KERNEL);
    if(newMessage==NULL)
    {
        printk("could not allocate a new message\n");
        return IRQ_HANDLED;
    }


    if(myrf_global==NULL)
    {
        printk("failed to access spi device\n");
        return -ENODEV;
    }
    mutex_lock(&myrf_global->lock);
    transfer.rx_buf=newMessage->message;
    spi_message_init(&message);
    spi_message_add_tail(&transfer,&message);
    value=spi_sync(myrf_global->spi,&message);
    if(value<0)
    {
        printk("message was not read\n");
        mutex_unlock(&myrf_global->lock);
 	    return IRQ_HANDLED;    
    }

    rfm69_write_register(myrf_global,REG_OPMODE,RF_OPMODE_SEQUENCER_ON|RF_OPMODE_LISTEN_OFF|RF_OPMODE_LISTENABORT|RF_OPMODE_STANDBY);
    rfm69_write_register(myrf_global,REG_OPMODE,RF_OPMODE_SEQUENCER_ON|RF_OPMODE_LISTEN_OFF|RF_OPMODE_STANDBY);
    rfm69_write_register(myrf_global,REG_OPMODE,RF_OPMODE_SEQUENCER_ON|RF_OPMODE_LISTEN_ON|RF_OPMODE_STANDBY);

    

    if(headList==NULL)
    {
        headList=newMessage;
        tailList=newMessage;
    }
    else
    {
        tailList->next=newMessage;
        tailList=newMessage;
    }



    //put here to write register to put back on listen mode



    info.si_signo = SIGUSR1;
	info.si_code = 1;
	info.si_int  = 1234;
    send_sig_info( SIGUSR1, &info,myrf_global->PID );
    //put message queue here
    //send signal here
    mutex_unlock(&myrf_global->lock);

 	return IRQ_HANDLED;

}
static const struct file_operations rfm_fops =
 {
	.owner =	THIS_MODULE,
    .open =		rfm69_open,
    .read =     rfm69_read,
	.write =	rfm69_write,

};

static struct spi_driver rfm69_driver =
{
    .driver=
    {
        .name = DRIVER_NAME,
       .bus  = &spi_bus_type, 
       .owner = THIS_MODULE,
    },
    .probe    = rfm69_probe,
    .remove   = rfm69_remove,
};

static int __init rfm69_init(void)
{
  int status;


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
    

    printk( "exit\n");

	spi_unregister_driver(&rfm69_driver);
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


