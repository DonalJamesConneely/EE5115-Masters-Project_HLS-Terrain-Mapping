#ifndef __SMULT_H__
#define __SMULT_H__

#include <stdio.h>
#include <assert.h>
#include <ap_axi_sdata.h>

//Set stream data type
typedef ap_axiu<32,1,1,1> stream_type;

void hls_mult(stream_type in_data_A[10], stream_type in_data_B[10], stream_type out_data[10]);

#endif

