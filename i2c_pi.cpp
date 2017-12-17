//g++ -std=c++11 -lpigpio -lpthread -lrt -o i2c_pi i2c_pi.cpp
//MAIS RECENTE: g++ -pthread -o i2c_pi i2c_pi.cpp -lpigpio -lrt

#include <stdio.h>
#include <unistd.h>
#include "pigpio.h"
#include <iostream>

#define SCL 19
#define SDA 18
#define ADDRESS 0x09 

bsc_xfer_t xfer;

void i2c(int event, uint32_t tick){
	
	int status;
	status = bscXfer(&xfer);
	
	if(xfer.rxCnt !=0)
	{
		std::cout<< xfer.rxBuf<< std::endl;
		std::cout<< "Recebido " << xfer.rxCnt << " bytes" << std::endl; 
		
	}
}
	


int main(int argc, char *argv[])
{
	int i,j;
	int status;
	
	/* Initialize Slave*/
	xfer.control = (0x09<<16) | 0x305;
	
	if (gpioInitialise() < 0) { printf("Erro 1\n"); return 1;}
	
	
	gpioSetPullUpDown(18, PI_PUD_UP);
	gpioSetPullUpDown(19, PI_PUD_UP);
	
	//gpioSetMode(SDA, PI_ALT3); //Dados
	//gpioSetMode(SCL, PI_ALT3); //Clock
	
	eventSetFunc(PI_EVENT_BSC, i2c);
	
	status = bscXfer(&xfer);
	 			   	
	sleep(600);
	
	xfer.control = 0;
	bscXfer(&xfer);
	
	gpioTerminate();
}
