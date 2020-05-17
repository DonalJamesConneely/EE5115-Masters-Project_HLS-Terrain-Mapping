#include "mult.h"
#include <stdio.h>

int main()
{
	printf("\n\n");
	int a, b, c;

	a = 10;
	b = 10;

	printf("Basic Multiplication\n");
	printf("%d * %d", a, b);

	mult(a, b, &c);
	printf(" = %d", c);

	printf("\n\n");
	return 0;
}
