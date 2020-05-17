#include <iostream>
#include "CoreSLAM.h"
#include "math.h"
#include "stdint.h"

#define SWAP(x,y)(x ^= y ^= x ^= y)

using namespace std;
static unsigned long val = 1;
static int initval = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2;

//Pseudo Random Number Generator (Based on C::rand() and LFSR)
int myrand()
{
	val = (val * 1103515245) + 12345;
	return((unsigned)(val / 65536) % RAND_MAX);
}

//Map Initialisation
void HLS_map_init(ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE])
{
	for (int i = 0; i < (TS_MAP_SIZE * TS_MAP_SIZE); i++)
	{
		map[i] = initval;
	}
}

//Distance Calculation
void HLS_distance(stream_scan scan[TS_SCAN_SIZE][4], ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE], stream_pos pos[3], int *output)
{
	float scanx, scany, posx, posy, postheta;
	float c, s;
	int i, x, y, nb_points = 0;
	int scanvalue, scannb_points;
	int64_t sum;

	posx = pos[0].data;
	posy = pos[1].data;
	postheta = pos[2].data;

	c = cosf(postheta * M_PI / 180);
	s = sinf(postheta * M_PI / 180);

	scannb_points = ((int)scan[0][0].data);

	//Translate and rotate scan to robot position and compute the distance.
	for (i = 0, sum = 0; i != scannb_points; i++)
	{
		scannb_points = ((int)scan[i][0].data);
		scanx = scan[i][1].data;
		scany = scan[i][2].data;
		scanvalue = (int)scan[i][3].data;

		if (scanvalue != TS_NO_OBSTACLE)
		{
			x = (int)floorf((posx + c * scanx - s * scany) * TS_MAP_SCALE + 0.5);
			y = (int)floorf((posy + s * scanx + c * scany) * TS_MAP_SCALE + 0.5);

			//Check Boundaries
			if (x >= 0 && x < TS_MAP_SIZE && y >= 0 && y < TS_MAP_SIZE) {
				sum += map[y * TS_MAP_SIZE + x];
				nb_points++;
			}
		}
	}
	if (nb_points) sum = sum * 1024 / nb_points;
	else sum = 2000000000;
	*output = (int)sum;
}

//Use the Monte Carlo Method
void HLS_monte_carlo(stream_scan scan[TS_SCAN_SIZE][4], ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE], stream_pos in_pos[3], stream_pos out_pos[3])
{
	int currentdist, bestdist, lastbestdist;
	int counter = 0;
	stream_pos curpos[3], bestpos[3], lastbestpos[3];
	stream_scan scanner[TS_SCAN_SIZE][4];

	for (int i = 0; i < 3; i++)
	{
		curpos[i].data = bestpos[i].data = lastbestpos[i].data = in_pos[i].data;
	}

	//Read Data so data is sequentially read.
	for (int i = 0; i < TS_SCAN_SIZE; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			scanner[i][j].data = scan[i][j].data;
		}
	}

	HLS_distance(scanner, map, curpos, &currentdist);
	bestdist = lastbestdist = currentdist;

	do {
		for (int i = 0; i < 3; i++)
		{
			curpos[i].data = lastbestpos[i].data;
		}

		curpos[0].data += 50 * (((double)myrand()) / RAND_MAX - 0.5);
		curpos[1].data += 50 * (((double)myrand()) / RAND_MAX - 0.5);
		curpos[2].data += 50 * (((double)myrand()) / RAND_MAX - 0.5);

		HLS_distance(scanner, map, curpos, &currentdist);

		if (currentdist < bestdist)
		{
			bestdist = currentdist;
			for (int i = 0; i < 3; i++)
			{
				bestpos[i].data = curpos[i].data;
			}
		}
		else
		{
			counter++;
		}
		if (counter > 100) {
			if (bestdist < lastbestdist)
			{
				for (int i = 0; i < 3; i++)
				{
					lastbestpos[i].data = bestpos[i].data;
				}
				lastbestdist = bestdist;
				counter = 0;
			}
		}
	} while (counter < 1000);

	//Transfer Back Position
	for (int i = 0; i < 3; i++)
	{
		out_pos[i].data = bestpos[i].data;
	}
}

//Laser Ray Scan
void HLS_map_laser_ray(ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE], int x1, int y1, int x2, int y2, int xp, int yp, int value, int alpha)
{
#pragma HLS allocation instances=SWAP limit=1 function
	int x2c, y2c, dx, dy, dxc, dyc, error, errorv, derrorv, x, ptr;
	int incv, sincv, incerrorv, incptrx, incptry, horiz, diago, pixval;

	if (x1 < 0 || x1 >= TS_MAP_SIZE || y1 < 0 || y1 >= TS_MAP_SIZE)
	{
		return;
	}

	x2c = x2;
	y2c = y2;

	//Clipping
	if (x2c < 0) {
		if (x1 == x2c) return;
		y2c += (y2c - y1) * (-x2c) / (x2c - x1);
		x2c = 0;
	}

	if (x2c >= TS_MAP_SIZE) {
		if (x1 == x2c) return;
		y2c += (y2c - y1) * (TS_MAP_SIZE - 1 - x2c) / (x2c - x1);
		x2c = TS_MAP_SIZE - 1;
	}

	if (y2c < 0) {
		if (y1 == y2c) return;
		x2c += (x1 - x2c) * (-y2c) / (y1 - y2c);
		y2c = 0;
	}

	if (y2c >= TS_MAP_SIZE) {
		if (y1 == y2c) return;
		x2c += (x1 - x2c) * (TS_MAP_SIZE - 1 - y2c) / (y1 - y2c);
		y2c = TS_MAP_SIZE - 1;
	}

	dx = abs(x2 - x1);
	dy = abs(y2 - y1);
	dxc = abs(x2c - x1);
	dyc = abs(y2c - y1);
	incptrx = (x2 > x1) ? 1 : -1;
	incptry = (y2 > y1) ? TS_MAP_SIZE : -TS_MAP_SIZE;
	sincv = (value > TS_NO_OBSTACLE) ? 1 : -1;

	if (dx > dy) {
		derrorv = abs(xp - x2);
	}
	else
	{
		SWAP(dx, dy);
		SWAP(dxc, dyc);
		SWAP(incptrx, incptry);
		derrorv = abs(yp - y2);
	}

	error = 2 * dyc - dxc;
	horiz = 2 * dyc;
	diago = 2 * (dyc - dxc);
	errorv = derrorv / 2;
	incv = (value - TS_NO_OBSTACLE) / derrorv;
	incerrorv = value - TS_NO_OBSTACLE - derrorv * incv;
	ptr = y1 * TS_MAP_SIZE + x1;
	pixval = TS_NO_OBSTACLE;

	for (x = 0; x <= dxc; x++, ptr += incptrx)
	{
		if (x > dx - 2 * derrorv) {
			if (x <= dx - derrorv) {
				pixval += incv;
				errorv += incerrorv;
				if (errorv > derrorv) {
					pixval += sincv;
					errorv -= derrorv;
				}
			}
			else {
				pixval -= incv;
				errorv -= incerrorv;
				if (errorv < 0) {
					pixval -= sincv;
					errorv += derrorv;
				}
			}
		}

		//Integration into Map
		map[ptr] = ((256 - alpha) * (map[ptr]) + alpha * pixval) >> 8;
		if (error > 0) {
			ptr += incptry;
			error += diago;
		}
		else error += horiz;
	}
}

//Map Update
void HLS_map_update(stream_scan scan[TS_SCAN_SIZE][4], ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE], stream_pos pos[3], int quality)
{
#pragma HLS allocation instances=HLS_map_laser_ray limit=1 function
	float c, s, q, x2p, y2p, add, dist;
	float scanx, scany, posx, posy, postheta;
	int i, x1, y1, x2, y2, xp, yp, value;
	int scanvalue, scannb_points;

	posx = pos[0].data;
	posy = pos[1].data;
	postheta = pos[2].data;

	c = cosf(postheta * M_PI / 180);
	s = sinf(postheta * M_PI / 180);
	x1 = (int)floorf(posx * TS_MAP_SCALE + 0.5);
	y1 = (int)floorf(posy * TS_MAP_SCALE + 0.5);

	scannb_points = ((int)scan[0][0].data);
	scanx = scan[0][1].data;
	scany = scan[0][2].data;
	scanvalue = ((int)scan[0][3].data);

	//Translate and rotate can to robot position
	for (i = 0; i != scannb_points; i++)
	{
		//Cannot Read Data from same location twice
		if (i > 0)
		{
			scannb_points = ((int)scan[i][0].data);
			scanx = scan[i][1].data;
			scany = scan[i][2].data;
			scanvalue = ((int)scan[i][3].data);
		}

		x2p = c * scanx - s * scany;
		y2p = s * scanx + c * scany;

		xp = (int)floorf((posx + x2p)* TS_MAP_SCALE + 0.5);
		yp = (int)floorf((posy + y2p)* TS_MAP_SCALE + 0.5);
		dist = sqrtf(x2p * x2p + y2p * y2p);

		add = TS_HOLE_WIDTH / 2 / dist;
		x2p *= TS_MAP_SCALE * (1 + add);
		y2p *= TS_MAP_SCALE * (1 + add);

		x2 = (int)floorf(posx * TS_MAP_SCALE + x2p + 0.5);
		y2 = (int)floorf(posy * TS_MAP_SCALE + y2p + 0.5);

		if (scanvalue == TS_NO_OBSTACLE)
		{
			q = quality / 2;
			value = TS_NO_OBSTACLE;
		}
		else
		{
			q = quality;
			value = TS_OBSTACLE;
		}

		HLS_map_laser_ray(map, x1, y1, x2, y2, xp, yp, value, q);
	}
}

//HLS Wrapper Function (Switch Based System)
void HLS_tinySLAM(stream_scan scan[TS_SCAN_SIZE][4], ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE], stream_pos in_pos[3], stream_pos out_pos[3], int *value, stream_type Run[1])
{
/*#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE s_axilite port=map
#pragma HLS INTERFACE axis port=Run
#pragma HLS INTERFACE axis port=scan
#pragma HLS INTERFACE axis port=in_pos
#pragma HLS INTERFACE axis port=out_pos
#pragma HLS INTERFACE s_axilite port=value
#pragma HLS allocation instances=cosf limit=1 function
#pragma HLS allocation instances=sinf limit=1 function
#pragma HLS allocation instances=abs limit=1 function
#pragma HLS inline region recursive*/

	int check = Run[0];

	//If Run == 1-> HLS_map_init
	if (check == 1)
	{
		HLS_map_init(map);
	}

	//If Run == 2-> HLS_distance
	if (check == 2)
	{
		//HLS_distance(scan, map, in_pos, value);
		HLS_monte_carlo(scan, map, in_pos, out_pos);
	}

	//If Run == 3-> HLS_map
	if (check == 3)
	{
		HLS_map_update(scan, map, in_pos, *value);
	}
}