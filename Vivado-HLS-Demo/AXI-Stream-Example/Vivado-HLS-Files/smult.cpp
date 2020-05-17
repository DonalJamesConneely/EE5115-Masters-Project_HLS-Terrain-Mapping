#include  "smult.h"
#include  <ap_axi_sdata.h>

void hls_mult(stream_type in_data_A[10], stream_type in_data_B[10], stream_type out_data[10])
{
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=in_data_A
#pragma HLS INTERFACE axis port=in_data_B
#pragma HLS INTERFACE axis port=out_data


	for(int i = 0; i < 10; i++)
	{
		//Set out_data.data = Answer
		out_data[i].data = in_data_A[i].data * in_data_B[i].data;

		//Set other AXI AP values equal to values of in_data_A
		out_data[i].strb = in_data_A[i].strb;
		out_data[i].keep = in_data_A[i].keep;
		out_data[i].user = in_data_A[i].user;
		out_data[i].last = in_data_A[i].last;
		out_data[i].id = in_data_A[i].id;
		out_data[i].dest = in_data_A[i].dest;
	}
}

