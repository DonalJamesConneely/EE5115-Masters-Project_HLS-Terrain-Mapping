#include <stdio.h>
#include <stdlib.h>
#include "smult.h"

int main()
{
	printf("Stream Multiplication\n");
	stream_type A[10], B[10], C[10];

	for(int i = 0; i < 10; i++)
	{
		A[i].data = i;
		A[i].strb = -1;
		A[i].keep = 15;
		A[i].user = 0;
		A[i].last = (i == 9) ? 1 : 0;
		A[i].id = 0;
		A[i].dest = 0;

		B[i].data = i;
		B[i].strb = -1;
		B[i].keep = 15;
		B[i].user = 0;
		B[i].last = (i == 9) ? 1 : 0;
		B[i].id = 0;
		B[i].dest = 0;
	}

	hls_mult(A, B, C);

	for(int i = 0; i < 10; i++)
	{
		printf("%d * %d = %d\n", (int) A[i].data, (int) B[i].data, (int) C[i].data);
	}

	return 0;
}

