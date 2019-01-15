#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>
#include <string.h>

#include "RDB_reader.h"
#include "RDB_error.h"

static char hexchars[] = "0123456789ABCDEF";

#ifdef USEHOST
#define READER_SERIAL_DEV "/dev/ttyUSB0"
#else
#define READER_SERIAL_DEV "/dev/ttyAMA0"
#endif

int unit_test(int fd);
unsigned char CheckSum(unsigned char *uBuff, unsigned char uBuffLen);
int host_cmd_reset(int fd);
int host_cmd_version(int fd, char *version);
int hex_to_char(unsigned char *bytes, char *hex, int size);
int reader_delay_sleep();

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
		printf("Failed to open serial port: %s\n", READER_SERIAL_DEV);
		return -1;
	}

	if( 0 > set_serial(fd, B115200))
		printf("Failed to setup serial port\n");

	unit_test(fd);
}


int host_cmd_reset(int fd)
{
	write(fd,&cmd_reset[0],cmd_reset[1]+2);
	return 0;
}

int host_cmd_version(int fd, char *version)
{
	unsigned char buf[64];

	write(fd,&cmd_version[0],cmd_version[1]+2);
    reader_delay_sleep();
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

	write(fd,&cmd_baud_38400[0],cmd_baud_38400[1]+2);
    reader_delay_sleep();
	read(fd,&buf,64);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_baudrate_115200(int fd)
{
	unsigned char buf[64];

	write(fd,&cmd_baud_115200[0],cmd_baud_115200[1]+2);
	reader_delay_sleep();
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
			break;
		case 2:
			cmd_antid_set[4] = 0x01;
			cmd_antid_set[5] = 0xE6;
			break;
		case 3:
			cmd_antid_set[4] = 0x02;
			cmd_antid_set[5] = 0xE5;
			break;
		case 4:
			cmd_antid_set[4] = 0x03;
			cmd_antid_set[5] = 0xE4;
			break;
		default:
			return CMD_ANT_ID_OOR;
	}
	write(fd,&cmd_antid_set[0],cmd_antid_set[1]+2);
	reader_delay_sleep();
	read(fd,&buf, resp_len);
	
	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_ant(int fd, int *antid)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_antid_get[0],cmd_antid_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];
	else
		*antid = buf[4] + 1;
	return 0;
}

int host_cmd_set_power(int fd, int power)
{
	unsigned char buf[64];
	int resp_len = 6;

	cmd_power_set[4] = power;
	cmd_power_set[5] = 0xE5 - power;

	write(fd,&cmd_power_set[0],cmd_power_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_power(int fd, int *power)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_power_get[0],cmd_power_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	*power = buf[4];

	return 0;
}

int host_cmd_set_region_NA(int fd)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_region_set[0],cmd_region_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_region(int fd, int *region)
{
	unsigned char buf[64];
	int resp_len = 8;

	write(fd,&cmd_region_get[0],cmd_region_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	*region = buf[4];

	return 0;
}

int host_cmd_get_temperature(int fd, int *temperature)
{
	unsigned char buf[64];
	int resp_len = 7;

	write(fd,&cmd_temp_get[0],cmd_temp_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);
	
	if(buf[4] == 0)
		*temperature = -buf[5];
	else if(buf[4] == 1)
		*temperature = buf[5];

	return 0;
}

int host_cmd_get_gpio(int fd, int *gpio1, int *gpio2)
{
	unsigned char buf[64];
	int resp_len = 7;

	write(fd,&cmd_gpio_get[0],cmd_gpio_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);
	
	*gpio1 = buf[4];
	*gpio2 = buf[5];

	return 0;
}

int host_cmd_set_gpio(int fd, int pin, int value)
{
	unsigned char buf[64];
	int resp_len = 6;

	if(pin == 0x03)
	{
		cmd_gpio_set[4] = 0x03;
		if(value == 0)
		{
			cmd_gpio_set[5] = 0x00;
			cmd_gpio_set[6] = 0xF6;
		}
		else	
		{
			cmd_gpio_set[5] = 0x01;
			cmd_gpio_set[6] = 0xF5;
		}
	}
	else if(pin == 0x04)
	{
		cmd_gpio_set[4] = 0x04;
		if(value == 0)
		{
			cmd_gpio_set[5] = 0x00;
			cmd_gpio_set[6] = 0xF5;
		}
		else	
		{
			cmd_gpio_set[5] = 0x01;
			cmd_gpio_set[6] = 0xF4;
		}
	}

	write(fd,&cmd_gpio_set[0],cmd_gpio_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);
	
	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}


int host_cmd_set_ant_detect(int fd)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_ant_det_set[0],cmd_ant_det_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_ant_detect(int fd, int *sensity)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_ant_det_get[0],cmd_ant_det_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	*sensity = buf[4];

	return 0;
}

int host_cmd_get_reader_id(int fd, char *readerID)
{
	unsigned char buf[64];
	int resp_len = 17;

	write(fd,&cmd_readerid_get[0],cmd_readerid_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	memcpy(readerID, (char *) &buf[4], 12);

	return 0;
}

int host_cmd_set_reader_id(int fd)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_readerid_set[0],cmd_readerid_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_set_rflink_profile(int fd, unsigned char profileID)
{
	unsigned char buf[64];
	int resp_len = 6;

	switch(profileID) {
		case 0xD0:
			cmd_rflink_prof_set[4] = 0xD0;
			cmd_rflink_prof_set[5] = 0x22;
			break;
		case 0xD1:
			cmd_rflink_prof_set[4] = 0xD1;
			cmd_rflink_prof_set[5] = 0x21;
			break;
		case 0xD2:
			cmd_rflink_prof_set[4] = 0xD2;
			cmd_rflink_prof_set[5] = 0x20;
			break;
		case 0xD3:
			cmd_rflink_prof_set[4] = 0xD3;
			cmd_rflink_prof_set[5] = 0x1F;
			break;
		default:
			break;
	}

	write(fd,&cmd_rflink_prof_set[0],cmd_rflink_prof_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_rflink_profile(int fd, unsigned char *profileID)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_rflink_prof_get[0],cmd_rflink_prof_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	*profileID = buf[4];

	return 0;
}

#define READ_BUF_SIZE	4096	
#define HEADER_SIZE		7	
int host_process_tag_read(int fd)
{
	unsigned char buf[READ_BUF_SIZE];
	unsigned char header[HEADER_SIZE];
	unsigned char body[64];
	int rbyte, databyte;

	rbyte = read(fd, &header[0], HEADER_SIZE);
	
	while(1)
	{
		if(header[1] == 0x0a)
		{	
			rbyte = read(fd, &body[0], header[1]);

			printf("Total tag read: %d with ant %d\n", body[3], body[1]); 
			break;
		}
		else if(rbyte > 0 && header[0] == 0xA0 && header[1] > 0x0a)
		{
		    printf("freq %02x ", header[4]);
			databyte = header[1] - 5;
			rbyte = read(fd, &body[0], databyte);
		
			for(int i=0; i < rbyte; i++)
				printf("%02x ", body[i]);

			printf(" rssi %d\n", body[rbyte-2]);
		}
		rbyte = read(fd, &header[0], HEADER_SIZE);
	}


}

int host_cmd_read_time_inventory(int fd)
{
	write(fd,&cmd_read_time_inv[0],cmd_read_time_inv[1]+2);
	reader_delay_sleep();
	
	return 0;
}

int reader_delay_sleep()
{
    usleep(150000);
}

int host_process_read_all_ants(int fd, RASPI_TAG_T *raspi_tag)
{
	unsigned char buf[READ_BUF_SIZE];
	unsigned char header[HEADER_SIZE];
	unsigned char epc[64];
	int rbyte, databyte, i;
	RDB_TAG_RESP_T *rdb_tag;
	int raspi_tag_cnt = 0;
	int location = 0;

	bzero(&buf[0],READ_BUF_SIZE); 
	rbyte = read(fd,&buf[0], READ_BUF_SIZE);

	for(i = 0; i < rbyte; i++)
	{
		if(buf[i] != 0xA0 && buf[i+3] != 0x8A)
		{
			//printf("%02x ", buf[i]);
		}
		else if(buf[i] == 0xA0 && buf[i+3] == 0x8A)
		{
			if(buf[i+1] < 16) //Less than EPC length + PC + CSUM + CMD + FREQ_ANT
			{
				i += buf[i+1] + 1;
				continue;
			}
			else
			{
				location = i+buf[i+1]-12; // cur_pos + len + header + EPC + rssi + csum
				bzero(&epc[0],64);
				hex_to_char(&buf[location],&epc[0],EPC_12BYTE);
				rdb_tag = (RDB_TAG_RESP_T *) &buf[i];
				// RFR Tag message EPC-LEN-RSSI-ANTID-FREQ	
				snprintf(raspi_tag->Tags[raspi_tag_cnt], TAG_SIZE, "%s %d %02d %02d %02d", //NO READ COUNT
					&epc[0], EPC_12BYTE, (buf[i+4]&0x03)+1, buf[location+EPC_12BYTE],buf[i+4]>>2 ); 

				printf("\n%s", raspi_tag->Tags[raspi_tag_cnt]);
				raspi_tag_cnt++;
			}
			//printf("\t%02x ", buf[i]);
		}
	}

	printf("\n Total tags: %d\n", raspi_tag_cnt);
	return 0;
}

int host_cmd_read_all_ants(int fd)
{
	write(fd,&cmd_read_all_ants[0],cmd_read_all_ants[1]+2);
	reader_delay_sleep();
	
	return 0;
}

int host_process_read_single_port(int fd, RASPI_TAG_T *raspi_tag)
{
	unsigned char buf[READ_BUF_SIZE];
	unsigned char header[HEADER_SIZE];
	unsigned char epc[64];
	int rbyte, databyte, i;
	RDB_TAG_RESP_T *rdb_tag;
	int raspi_tag_cnt = 0;
	int location = 0;

	bzero(&buf[0],READ_BUF_SIZE); 
	rbyte = read(fd,&buf[0], READ_BUF_SIZE);

	for(i = 0; i < rbyte; i++)
	{
		if(buf[i] != 0xA0 && buf[i+3] != 0x8B)
		{
			printf("%02x ", buf[i]);
		}
		else if(buf[i] == 0xA0 && buf[i+3] == 0x8B)
		{
			if(buf[i+1] < 16) //Less than EPC length + PC + CSUM + CMD + FREQ_ANT
			{
				i += buf[i+1] + 1;
				continue;
			}
			else
			{
				location = i+buf[i+1]-12; // cur_pos + len + header + EPC + rssi + csum
				bzero(&epc[0],64);
				hex_to_char(&buf[location],&epc[0],EPC_12BYTE);
				rdb_tag = (RDB_TAG_RESP_T *) &buf[i];
				// RFR Tag message EPC-LEN-RSSI-ANTID-FREQ	
				snprintf(raspi_tag->Tags[raspi_tag_cnt], TAG_SIZE, "%s %d %02d %02d %02d", //NO READ COUNT
					&epc[0], EPC_12BYTE, (buf[i+4]&0x03)+1, buf[location+EPC_12BYTE],buf[i+4]>>2 ); 

				printf("\n%s", raspi_tag->Tags[raspi_tag_cnt]);
				raspi_tag_cnt++;
			}
			printf("\t%02x ", buf[i]);
		}
	}
	printf("\n Total tags: %d\n", raspi_tag_cnt);

    return 0;
}

int host_cmd_read_single_port(int fd)
{
        write(fd,&cmd_read_single_port[0],cmd_read_single_port[1]+2);
        reader_delay_sleep();

        return 0;
}


int hex_print(unsigned char *cmd, int len)
{
	int i;
	for(int i=0; i < len; i++)
		printf("%02x ", *cmd++);

	printf("\n");
}

int hex_to_char(unsigned char *bytes, char *hex, int size)
{
	while (size--)
    {
		*hex++ = hexchars[*bytes >> 4];
        *hex++ = hexchars[*bytes & 15];
        bytes++;
    }
    *hex = '\0';
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
	int error;
	RASPI_TAG_T raspi_tag;
	// reset
	//host_cmd_reset(fd);

	// get version
	//host_cmd_version(fd,&version[0]);
	//printf("%s\n", &version[0]);

	// ant ID
	//host_cmd_set_ant(fd, 3);

	//int getantid;
	//host_cmd_get_ant(fd, &getantid);
	//printf("Get antid: %d\n", getantid+1);

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
	//int temperature;
	//host_cmd_get_temperature(fd, &temperature);
	//printf("Get temperature: %d\n", temperature);

	// gpio get
	//int gpio1, gpio2;
	//host_cmd_get_gpio(fd, &gpio1, &gpio2);
	//printf("Get gpio1: %d gpio2: %d\n", gpio1, gpio2);

	// gpio set
	//host_cmd_set_gpio(rd, 0x03, 1);

	// antenna detect set
	//host_cmd_set_ant_detect(fd);

	// set reader ID
	//host_cmd_set_reader_id(fd);
	//printf("Done reader id set\n");

	// get reader ID
	//unsigned char readerID[16];
	//host_cmd_get_reader_id(fd, &readerID[0]);
	//for(int i=0; i < 12; i++)
	//	printf("0x%02x ", readerID[i]);

	// set rflink profile
	//host_cmd_set_rflink_profile(fd, 0x0D);

	// get rflink profile
	//unsigned char profileID;
	//host_cmd_get_rflink_profile(fd, &profileID);
	//printf("Get profileID: 0x%02x\n", profileID);

	// To read set antenna then read
	//error = host_cmd_set_ant(fd, 0x00);
	//if(error > 0)
	//	printf("Set ant failed error: %d\n", error);
	//host_cmd_read_time_inventory(fd);
	//host_process_tag_read(fd);
	
	// To read set antenna then read
	//host_cmd_read_all_ants(fd);
	//host_process_read_all_ants(fd, &raspi_tag);


	// Single port reader module
for(;;)
{
    host_cmd_set_ant(fd, 0);
    host_cmd_read_single_port(fd);
    host_process_read_single_port(fd, &raspi_tag);
}


}
