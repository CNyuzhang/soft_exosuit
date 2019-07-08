#ifndef __LRN_H__
#define __LRN_H__
#include "sys.h"

struct lrn{
		float Kp;
		float Kd;
		float D;
		float bata;
		float Kl;
		float mu;
		float error;
};



#endif
