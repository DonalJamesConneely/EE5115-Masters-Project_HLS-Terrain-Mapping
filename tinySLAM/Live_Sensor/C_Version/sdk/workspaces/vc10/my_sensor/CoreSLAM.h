#ifndef _CoreSLAM_H_
#define _CoreSLAM_H_

#include <stdio.h>
#include <assert.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*#define TS_SCAN_SIZE 683
#define TS_MAP_SIZE 2048
#define TS_MAP_SCALE 0.1
#define TS_DISTANCE_NO_DETECTION 4000
#define TS_NO_OBSTACLE 65500
#define TS_OBSTACLE 0
#define TS_HOLE_WIDTH 600*/

#define TS_SCAN_SIZE 683
#define TS_MAP_SIZE 360
#define TS_MAP_SCALE 0.017578125
#define TS_DISTANCE_NO_DETECTION 4000
#define TS_NO_OBSTACLE 65500
#define TS_OBSTACLE 0
#define TS_HOLE_WIDTH 600

typedef unsigned short ts_map_pixel_t;

typedef struct {
	ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE];
} ts_map_t;

typedef struct {
	float x[TS_SCAN_SIZE];
	float y[TS_SCAN_SIZE];
	int value[TS_SCAN_SIZE];
	int nb_points;
} ts_scan_t;

typedef struct {
	float x; // in mm
	float y; // in mm
	float theta; // in degree s
} ts_position_t;


template<int D, int U, int TI, int TD>
struct ap_scan {
	float data;
	int keep;
	int strb;
	int user;
	int last;
	int id;
	int dest;
};

template<int D, int U, int TI, int TD>
struct ap_position {
	float data;
	int keep;
	int strb;
	int user;
	int last;
	int id;
	int dest;
};

typedef int stream_type;
typedef ap_scan<32, 1, 1, 1> stream_scan;
typedef ap_position<32, 1, 1, 1> stream_pos;

void HLS_tinySLAM(stream_scan scan[TS_SCAN_SIZE][4], ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE], stream_pos in_pos[3], stream_pos out_pos[3], int *value, stream_type Run[1]);
#endif