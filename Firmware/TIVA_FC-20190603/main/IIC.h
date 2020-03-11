#ifndef __IIC_H
#define __IIC_H

#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_i2c.h"
#include "i2c.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"


void IIC_INIT(void);
int i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data);
int i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);
void delay_ms(unsigned int n);
void get_ms(unsigned long *time);

#endif
