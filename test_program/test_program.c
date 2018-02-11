// #include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>



static const char *device = "/dev/rfm69_0.0";

int main()
{
	int ret = 0;
	int fd;




	fd = open(device, O_RDWR);
	if (fd < 0)
    {
        printf("error\n");
        return -22;
    }
    printf("ok\n");
        
    close(fd);

	return ret;
}