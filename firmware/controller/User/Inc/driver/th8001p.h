#ifndef __TH8001P_H__
#define __TH8001P_H__

#include "main.h"

typedef struct {
    int adc_val[4];
} th8001p_Data;

void th8001p_init(void);
void th8001p_read_data(th8001p_Data* data);

#endif
