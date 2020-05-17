#include "mult.h"
#include <stdio.h>

void mult(int A, int B, int *C)
{
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE s_axilite port=A
#pragma HLS INTERFACE s_axilite port=B
#pragma HLS INTERFACE s_axilite port=C

    *C = A * B;
}
