#include <linux/module.h>
#include <asm/io.h>
#include <mach/platform.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>

#define DEVICE_NAME "ESRGled"
#define CLASS_NAME "ESRGledClass"

MODULE_LICENSE("GPL");

/* Device variables */
static struct class* ESRGledDevice_class = NULL;
static struct device* ESRGledDevice_device = NULL;
static dev_t ESRGledDevice_majorminor;
static struct cdev c_dev;  // Character device structure

struct GpioRegisters
{
	uint32_t GPFSEL[6];
	uint32_t Reserved1;
	uint32_t GPSET[2];
	uint32_t Reserved2;
	uint32_t GPCLR[2];
};

static struct class *s_pDeviceClass;
static struct device *s_pDeviceObject;
struct GpioRegisters *s_pGpioRegisters;
static const int LedGpioPin = 29;

static void SetGPIOFunction(int GPIO, int functionCode) {
	int registerIndex = GPIO / 10;
	int bit = (GPIO % 10) * 3;

	unsigned oldValue = s_pGpioRegisters->GPFSEL[registerIndex];
	unsigned mask = 0b111 << bit;

	s_pGpioRegisters->GPFSEL[registerIndex] = (oldValue & ~mask) | ((functionCode << bit) & mask);
}

static void SetGPIOOutputValue(int GPIO, bool outputValue) {
	if (outputValue)
		s_pGpioRegisters->GPSET[GPIO / 32] = (1 << (GPIO % 32));
	else
		s_pGpioRegisters->GPCLR[GPIO / 32] = (1 << (GPIO % 32));
}

static ssize_t ESRGledDeviceWrite(struct file *f, const char __user *buf, size_t len, loff_t *off) {
	if (buf[0]=='0')
		SetGPIOOutputValue(LedGpioPin, 0);
	else
		SetGPIOOutputValue(LedGpioPin, 1);
	return len;
}

static struct file_operations ESRGledDevice_fops = {
	.owner = THIS_MODULE,
	.write = ESRGledDeviceWrite
};

static int __init ESRGledModule_init(void) {
	int ret;
	struct device *dev_ret;

	if ((ret = alloc_chrdev_region(&ESRGledDevice_majorminor, 0, 1, DEVICE_NAME)) < 0) {
		return ret;
	}

	if (IS_ERR(ESRGledDevice_class = class_create(THIS_MODULE, CLASS_NAME))) {
		unregister_chrdev_region(ESRGledDevice_majorminor, 1);
		return PTR_ERR(ESRGledDevice_class);
	}
	if (IS_ERR(dev_ret = device_create(ESRGledDevice_class, NULL, ESRGledDevice_majorminor, NULL, DEVICE_NAME))) {
		class_destroy(ESRGledDevice_class);
		unregister_chrdev_region(ESRGledDevice_majorminor, 1);
		return PTR_ERR(dev_ret);
	}
	
	cdev_init(&c_dev, &ESRGledDevice_fops);
	c_dev.owner = THIS_MODULE;
	if ((ret = cdev_add(&c_dev, ESRGledDevice_majorminor, 1)) < 0) {
		printk(KERN_NOTICE "Error %d adding device", ret);
		device_destroy(ESRGledDevice_class, ESRGledDevice_majorminor);
		class_destroy(ESRGledDevice_class);
		unregister_chrdev_region(ESRGledDevice_majorminor, 1);
		return ret;
	}

	s_pGpioRegisters = (struct GpioRegisters *) __io_address(GPIO_BASE);
	SetGPIOFunction(LedGpioPin, 0b001); //Output

	return 0;
}

static void __exit ESRGledModule_exit(void) {
	SetGPIOFunction(LedGpioPin, 0); //Configure the pin as input
	cdev_del(&c_dev);
	device_destroy(ESRGledDevice_class, ESRGledDevice_majorminor);
	class_destroy(ESRGledDevice_class);
	unregister_chrdev_region(ESRGledDevice_majorminor, 1);
}

module_init(ESRGledModule_init);
module_exit(ESRGledModule_exit);

