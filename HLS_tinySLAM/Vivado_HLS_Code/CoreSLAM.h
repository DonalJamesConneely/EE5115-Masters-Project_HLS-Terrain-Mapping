#ifndef _CoreSLAM_H_
#define _CoreSLAM_H_

#include <stdio.h>
#include <assert.h>
#include <ap_axi_sdata.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TS_SCAN_SIZE 683
#define TS_MAP_SIZE 360
#define TS_MAP_SCALE 0.017578125
#define TS_DISTANCE_NO_DETECTION 4000
#define TS_NO_OBSTACLE 65500
#define TS_OBSTACLE 0
#define TS_HOLE_WIDTH 600

typedef unsigned short ts_map_pixel_t;

template<int D,int U,int TI,int TD>
struct ap_position{
	float data;
	ap_uint<D/8> keep;
	ap_uint<D/8> strb;
	ap_uint<U> user;
	ap_uint<1> last;
	ap_uint<TI> id;
	ap_uint<TD> dest;
};

typedef ap_axiu<32,1,1,1> stream_type;
typedef ap_position<32,1,1,1> stream_pos;

void HLS_tinySLAM(int scan[TS_SCAN_SIZE][4], ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE], stream_pos in_pos[3], stream_pos out_pos[3], stream_type Run[1]);
#endif
