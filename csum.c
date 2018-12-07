
#include <stdio.h>
#include <string.h>

unsigned char cmd[] = {0xA0,0x03,0x01,0x7B};

int main()
{
    unsigned char i,uSum=0;
	int uBuffLen;

	uBuffLen = strlen(&cmd[0]);	
    for(i=0;i<uBuffLen;i++)
    {
        uSum = uSum + cmd[i];
    }
    uSum = (~uSum) + 1;

	printf("CS on CMD 0xx%02x 0x%02x\n", cmd[3], uSum);
    return uSum;
}

