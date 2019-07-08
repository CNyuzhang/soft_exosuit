#ifndef __COPLEY_NODE2_H__
#define __COPLEY_NODE2_H__
#include "copley_control.h"
extern copley_device copley_node2;
int node2_set_target(uint8_t target, int32_t val);
void node2_ppm_set_target(int target);
void node2_halt(void);

#define NODE2_DEBUG 0
#endif
