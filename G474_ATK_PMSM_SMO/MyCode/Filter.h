#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>


typedef struct
{
    float prev_output;
    uint8_t initialized;
    
} LPF1_t;   //Ò»½×µÍÍ¨ÂË²¨



void  LPF1_Reset(LPF1_t *filter , float initial_output);
float LPF1_Update(LPF1_t *filter, float input, float a);






#endif

