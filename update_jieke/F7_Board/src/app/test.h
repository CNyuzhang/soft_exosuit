#ifndef __TEST_H__
#define __TEST_H__
#include "sys.h"
void set_start(int flag);
void set_start2(int flag);
void set_kd(int kd);
void set_kp(int kp);
void show_pid(void);
void read_sdo(uint16_t index,uint8_t subindex);
void set_sdo(void);
void set_prepare_param(void);
#endif
