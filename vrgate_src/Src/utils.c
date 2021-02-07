/*
 * utils.c
 *
 *  Created on: 2020年8月21日
 *      Author: SalimTerryLi
 */


#include "utils.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "usbd_cdc_if.h"


uint16_t str2u16(const char * buf,const uint16_t size){
	uint16_t ret=0;
	uint8_t numbers[16];
	uint8_t width=0;

	for(uint16_t i=0;i<size;++i){
		if(buf[i]>='0' && buf[i]<='9'){
			numbers[width]=buf[i]-'0';
			++width;
		}else{
			break;
		}
	}
	for(uint8_t i=0;i<width;++i){
		ret=ret+numbers[i]*(uint16_t)pow(10,width-i-1);
	}
	return ret;
}

void heartbeat(){
	char uartbuf[1]={0xac};
	CDC_Transmit_FS((uint8_t*)uartbuf,1);
}
