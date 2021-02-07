/*
 * utils.h
 *
 *  Created on: 2020年8月21日
 *      Author: SalimTerryLi
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

typedef union{
	uint32_t u32;
	uint8_t u8[4];
} u32tou8;

void setPWMPulse(uint16_t pulseWidth);

void setInitMode(uint8_t mode);

void updateBcaklight(uint16_t val);

uint8_t checkSysReg();
uint8_t checkSysRegA(uint16_t addr);

void heartbeat();

uint16_t str2u16(const char * buf,const uint16_t size);

#endif /* UTILS_H_ */
