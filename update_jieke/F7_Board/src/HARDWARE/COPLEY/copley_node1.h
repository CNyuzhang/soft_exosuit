#ifndef __COPLEY_NODE1_H__
#define __COPLEY_NODE1_H__
#include "copley_control.h"
extern copley_device copley_node1;
int node1_set_target(uint8_t target, int32_t val);
void node1_ppm_set_target(int target);
void node1_halt(void);

#define NODE1_DEBUG 0

#endif
