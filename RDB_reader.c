#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

#include "RDB_reader.h"
#include "RDB_error.h"

#define READER_SERIAL_DEV "/dev/ttyUSB0"

int unit_test(int fd);
unsigned char CheckSum(unsigned char *uBuff, unsigned char uBuffLen);
int host_cmd_reset(int fd);
int host_cmd_version(int fd, char *version);

int set_serial(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

int main()
{
	int fd = -1, wbyte,rbyte;
	unsigned char cmd[64];
	unsigned char rsp[64];
	char version[16];

	fd = open(READER_SERIAL_DEV, O_RDWR);
	if( fd < 0)
	{
		//logging(LOG_ERROR, "Serial port open failed\n");
		printf("Failed to open serial port\n");
		return -1;
	}

	if( 0 > set_serial(fd, B115200))
		printf("Failed to setup serial port\n");

	unit_test(fd);
}


int host_cmd_reset(int fd)
{
	write(fd,&cmd_reset[0],strlen(&cmd_reset[0]));
	return 0;
}

int host_cmd_version(int fd, char *version)
{
	unsigned char buf[64];

	write(fd,&cmd_version[0],strlen(&cmd_version[0]));
    sleep(1);
	read(fd,&buf,64);

	*version = buf[4] + 0x30;
	*(version+1) = '.';
	*(version+2) = buf[5] + 0x30;
	*(version+3) = '\0';
	
	return 0;
}

int host_cmd_baudrate_38400(int fd)
{
	unsigned char buf[64];

	write(fd,&cmd_baud_38400[0],strlen(&cmd_baud_38400[0]));
    sleep(1);
	read(fd,&buf,64);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_baudrate_115200(int fd)
{
	unsigned char buf[64];

	write(fd,&cmd_baud_115200[0],strlen(&cmd_baud_115200[0]));
    sleep(1);
	read(fd,&buf,64);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;

}

int host_cmd_set_ant(int fd, int antid)
{
	unsigned char buf[64];
	int resp_len = 6;
	switch(antid) {
		case 1:
			cmd_antid_set[4] = 0x00;
			cmd_antid_set[5] = 0xE7;
			write(fd,&cmd_antid_set[0],strlen(&cmd_antid_set[0]));
			break;
		case 2:
			cmd_antid_set[4] = 0x01;
			cmd_antid_set[5] = 0xE6;
			write(fd,&cmd_antid_set[0],strlen(&cmd_antid_set[0]));
			break;
		case 3:
			cmd_antid_set[4] = 0x02;
			cmd_antid_set[5] = 0xE5;
			write(fd,&cmd_antid_set[0],strlen(&cmd_antid_set[0]));
			break;
		case 4:
			cmd_antid_set[4] = 0x03;
			cmd_antid_set[5] = 0xE4;
			write(fd,&cmd_antid_set[0],strlen(&cmd_antid_set[0]));
			break;
		default:
			return CMD_ANT_ID_OOR;
	}
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_ant(int fd, int *antid)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_antid_get[0],strlen(&cmd_antid_get[0]));
	sleep(1);
	
	read(fd,&buf, resp_len);

	*antid = buf[4] + 1;
	return 0;
}

int host_cmd_set_power(int fd, int power)
{
	unsigned char buf[64];
	int resp_len = 6;

	cmd_power_set[4] = power;
	cmd_power_set[5] = 0xE5 - power;

	write(fd,&cmd_power_set[0],strlen(&cmd_power_set[0]));
	sleep(1);
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_power(int fd, int *power)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_power_get[0],strlen(&cmd_power_get[0]));
	sleep(1);
	
	read(fd,&buf, resp_len);

	*power = buf[4];

	return 0;
}

int host_cmd_set_region_NA(int fd)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_region_set[0],strlen(&cmd_region_set[0]));
	sleep(1);
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_region(int fd, int *region)
{
	unsigned char buf[64];
	int resp_len = 8;

	write(fd,&cmd_region_get[0],strlen(&cmd_region_get[0]));
	sleep(1);
	
	read(fd,&buf, resp_len);

	*region = buf[4];

	return 0;
}

int host_cmd_get_temperature(int fd, int *temperature)
{
	unsigned char buf[64];
	int resp_len = 7;

	write(fd,&cmd_temp_get[0],strlen(&cmd_temp_get[0]));
	sleep(1);
	
	read(fd,&buf, resp_len);
	for(int i=0; i < resp_len; i++)
		printf("0x%02x ", buf[i]);
	
	if(buf[4] == 0)
		*temperature = -buf[5];
	else if(buf[4] == 1)
		*temperature = buf[5];

	return 0;
}





//	for(int i=0; i < resp_len; i++)
//		printf("0x%02x ", buf[i]);


unsigned char CheckSum(unsigned char *uBuff, unsigned char uBuffLen)
{
	unsigned char i,uSum=0;
	for(i=0;i<uBuffLen;i++)
	{
		uSum = uSum + uBuff[i];
	}
	uSum = (~uSum) + 1;
	return uSum;
}

int	unit_test(int fd)
{
	char version[16];
	// reset
	//host_cmd_reset(fd);

	// get version
	//host_cmd_version(fd,&version[0]);
	//printf("%s\n", &version[0]);

	// ant ID
	//int antid;
	//host_cmd_get_ant(fd, &antid);
	//printf("Get antid: %d\n", antid);

	// set power
	//host_cmd_set_power(fd, 33);

	// get power
	//int power;
	//host_cmd_get_power(fd,&power);
	//printf("Get ant power: %d\n", power);

	// Freq region NA set
	//host_cmd_set_region_NA(fd);

	// region get
	//int region;
	//host_cmd_get_region(fd, &region);
	//printf("Get region: %d\n", region);

	// temperature get
	int temperature;
	host_cmd_get_temperature(fd, &temperature);
	printf("Get temperature: %d\n", temperature);
}