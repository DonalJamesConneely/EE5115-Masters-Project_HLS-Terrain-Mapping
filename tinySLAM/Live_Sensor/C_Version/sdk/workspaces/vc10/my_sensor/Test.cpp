#include "windows.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "rplidar.h"
#include "CoreSLAM.h"
#include <signal.h>

#define TEST_FILENAME "test_lab.dat"
#define TEST_SCAN_SIZE 682
#define TEST_MIN_DIST 20
#define TEST_ANGLE_MIN -180
#define TEST_ANGLE_MAX 180
#define TEST_OFFSET_LASER 145
#define TEST_HOLE_WIDTH 600

using namespace cv;
using namespace rp::standalone::rplidar;

//Parameter of Robot
typedef struct {
	double r;	    // length wheels' radius
	double R;	    // half the wheels' axis length
	int inc;	    // wheels' counters increments per turn
	double ratio;   // ratio between left and right wheel
} cart_parameters_t;

//Sensor Data
typedef struct {
	int timestamp;
	int q1, q2;
	ts_scan_t scan;
} ts_sensor_data_t;

//Global Variable
ts_sensor_data_t sensor_data[600];
ts_map_t trajectory, map;
RPlidarDriver *drv;
bool ctrl_c_pressed;
const unsigned short MSB = 0x8000;

void ctrlc(int)
{
	ctrl_c_pressed = true;
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;


	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		//printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			return false;
		}
		else {
			return true;
		}

	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}

bool StartSensor()
{
	const char * opt_com_path = NULL;
	_u32         baudrateArray[2] = { 115200, 256000 };
	_u32         opt_com_baudrate = 0;
	u_result     op_result;
	double angle_deg, angle_rad;
	bool setup = true;

	opt_com_path = "\\\\.\\com3";

	// create the driver instance
	drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}

	rplidar_response_device_info_t devinfo;
	bool connectSuccess = false;

	size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
	for (size_t i = 0; i < baudRateArraySize; ++i)
	{
		if (!drv)
			drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
		if (IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
		{
			op_result = drv->getDeviceInfo(devinfo);

			if (IS_OK(op_result))
			{
				connectSuccess = true;
				break;
			}
			else
			{
				delete drv;
				drv = NULL;
			}
		}
	}

	if (!connectSuccess)
	{
		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_com_path);
		setup = false;
	}
	else if (!checkRPLIDARHealth(drv))
	{
		printf("LIDAR Health Error\n");
		setup = false;
	}
	else
	{
		drv->startMotor();
		drv->startScan(0, 1);
	}

	return setup;
}

void StopSensor()
{
	drv->stop();
	drv->stopMotor();

	RPlidarDriver::DisposeDriver(drv);
	drv = NULL;
}

void read_scan(ts_scan_t *scan)
{
	u_result     op_result;
	double angle_deg, angle_rad;

	// fetech result and print it out...
	rplidar_response_measurement_node_hq_t nodes[8192];
	size_t   count = _countof(nodes);

	//Get the scan
	float map[TEST_SCAN_SIZE];
	op_result = drv->grabScanDataHq(nodes, count);

	if (IS_OK(op_result))
	{
		drv->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count; ++pos)
		{
			map[pos] = (nodes[pos].dist_mm_q2 / 4.0f);
		}
	}

	scan->nb_points = 0;
#define SPAN 1
	// Span the laser scans to better cover the space
	for (int i = 0; i < TEST_SCAN_SIZE; i++) {
		for (int j = 0; j != SPAN; j++) {
			angle_deg = TEST_ANGLE_MIN + ((double)(i * SPAN + j)) * (TEST_ANGLE_MAX - TEST_ANGLE_MIN) / (TEST_SCAN_SIZE * SPAN - 1);
			angle_rad = angle_deg * M_PI / 180;

			if (i > 45 && i < TEST_SCAN_SIZE - 45) {
				if (map[i] == 0) {
					scan->x[scan->nb_points] = TS_DISTANCE_NO_DETECTION * cos(angle_rad);
					scan->y[scan->nb_points] = TS_DISTANCE_NO_DETECTION * sin(angle_rad);
					scan->value[scan->nb_points] = TS_NO_OBSTACLE;
					scan->x[scan->nb_points] += TEST_OFFSET_LASER;
					scan->nb_points++;
				}
				if (map[i] > TEST_HOLE_WIDTH / 2) {
					scan->x[scan->nb_points] = map[i] * cos(angle_rad);
					scan->y[scan->nb_points] = map[i] * sin(angle_rad);
					scan->value[scan->nb_points] = TS_OBSTACLE;
					scan->x[scan->nb_points] += TEST_OFFSET_LASER;
					scan->nb_points++;
				}
			}
		}
	}
}


//Save Map as a Portable Gray Map (PGM) Image file
void record_map(ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE], ts_map_pixel_t overlay[TS_MAP_SIZE * TS_MAP_SIZE], char *filename, int width, int height)
{
	int x, y, xp, yp;
	FILE *output;
	output = fopen(filename, "wt");

	//Create Columns
	fprintf(output, "P2\n%d %d 255\n", width, height);

	//Loop Through Rows (Y-Axis)
	y = (TS_MAP_SIZE - height) / 2;
	for (yp = 0; yp < height; y++, yp++)
	{
		//Loop Through Columns (X-Axis)
		x = (TS_MAP_SIZE - width) / 2;
		for (xp = 0; xp < width; x++, xp++)
		{
			//If nothing at that location print 0.
			if (overlay[(TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 0)
				fprintf(output, "0 ");

			//If value in location. Value equals number/(2^8).
			else
				fprintf(output, "%d ", (int)(map[(TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8);
		}
		fprintf(output, "\n");
	}
	fclose(output);
}

//Calculate New Map by updating data points
void draw_scan(stream_scan scan[TS_SCAN_SIZE][4], ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE], stream_pos pos[3])
{
	double c, s;
	double x2p, y2p;
	int i, x1, y1, x2, y2;
	double xs, ys;
	int vs;

	c = cos(pos[2].data * M_PI / 180);
	s = sin(pos[2].data * M_PI / 180);
	x1 = (int)floorf(pos[0].data * TS_MAP_SCALE + 0.5);
	y1 = (int)floorf(pos[1].data * TS_MAP_SCALE + 0.5);

	// Translate and rotate scan to robot position
	for (i = 0; i != scan[i][0].data; i++) {
		xs = scan[i][1].data;
		ys = scan[i][2].data;
		vs = scan[i][3].data;

		if (vs != TS_NO_OBSTACLE) {
			x2p = c * xs - s * ys;
			y2p = s * xs + c * ys;
			x2p *= TS_MAP_SCALE;
			y2p *= TS_MAP_SCALE;
			x2 = (int)floorf(pos[0].data * TS_MAP_SCALE + x2p + 0.5);
			y2 = (int)floorf(pos[1].data * TS_MAP_SCALE + y2p + 0.5);
			if ((x2 >= 0) && (y2 >= 0) && (x2 < TS_MAP_SIZE) && (y2 < TS_MAP_SIZE))
				map[y2 * TS_MAP_SIZE + x2] = 0;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////// MAIN ////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tinySLAM()
{
	//Initialise Variables
	FILE *output;
	char filename[256];
	int nb_sensor_data, told, x, y, value;
	int Run[1];
	stream_scan scaner[TS_SCAN_SIZE][4];
	stream_pos pos[3], pos2[3];
	ts_scan_t scan;
	static ts_map_pixel_t  map[TS_MAP_SIZE * TS_MAP_SIZE];
	static ts_map_pixel_t  traj[TS_MAP_SIZE * TS_MAP_SIZE];

	////////////////////////////////////Initial machine pose//////////////////////////////////////
	pos[0].data = 0.3 * TS_MAP_SIZE / TS_MAP_SCALE;
	pos[1].data = 0.3 * TS_MAP_SIZE / TS_MAP_SCALE;
	pos[2].data = 0;

	Run[0] = 1;
	HLS_tinySLAM(scaner, map, pos, pos2, &value, Run);
	HLS_tinySLAM(scaner, traj, pos, pos2, &value, Run);

	output = fopen("test_trajectory.dat", "wt");

	int cnt_scans = 0;
	if (StartSensor())
	{
		//Begin Loop
		bool go = true;
		while(go)
		{
			read_scan(&scan);
			for (int i = 0; i < TS_SCAN_SIZE; i++)
			{
				scaner[i][0].data = (float)scan.nb_points;
				scaner[i][1].data = scan.x[i];
				scaner[i][2].data = scan.y[i];
				scaner[i][3].data = (float)scan.value[i];
			}

			//Perform Monte-Carlo calculations
			Run[0] = 2;
			value = 0;
			HLS_tinySLAM(scaner, map, pos, pos2, &value, Run);

			/////////////////////////////////Update Map////////////////////////////////////////////
			for (int i = 0; i < 3; i++)
			{
				pos[i].data = pos2[i].data;
			}
			printf("#%d : %lg %lg %lg\n", cnt_scans, pos[0].data, pos[1].data, pos[2].data);

			Run[0] = 3;
			value = 50;
			HLS_tinySLAM(scaner, map, pos, pos2, &value, Run);

			///////////////////////////Set Current Pose as Obstacle Point//////////////////////////
			x = (int)floorf(pos[0].data * TS_MAP_SCALE + 0.5);
			y = ((int)floorf(pos[1].data * TS_MAP_SCALE + 0.5));

			if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
				traj[y * TS_MAP_SIZE + x] = 0;

			///////////////////////////Update the obstacle point map///////////////////////////////
			draw_scan(scaner, traj, pos);

			if (cnt_scans % 100 == 0)
			{
				sprintf(filename, "test_lab%04d.png", cnt_scans);
				record_map(map, traj, filename, TS_MAP_SIZE, TS_MAP_SIZE);

				//Display Map
				Mat img = imread(filename);
				circle(img, CvPoint(x, TS_MAP_SIZE - y), 5, (0, 0, 255), 3);
				namedWindow("image", WINDOW_NORMAL);
				imshow("image", img);
				waitKey(1000);
				destroyWindow("image");
			}
			fprintf(output, "\n");

			cnt_scans++;

			//Pressed and Hold the Escape Key to Exit the Loop
			if (GetAsyncKeyState(VK_ESCAPE) & MSB)
			{
				go = false;
			}
		}
		fclose(output);
		system("pause");

		// Record the map
		sprintf(filename, "test_lab_reversed.png");
		record_map(map, traj, filename, TS_MAP_SIZE, TS_MAP_SIZE);

		//Display Map
		Mat img = imread(filename);
		namedWindow("image", WINDOW_NORMAL);
		imshow("image", img);
		waitKey(0);

		circle(img, CvPoint(x, TS_MAP_SIZE - y), 5, (0, 0, 255), 3);
		imshow("image", img);
		waitKey(0);
		destroyWindow("image");

		StopSensor();
	}
	else
	{
		//If sensor error print error message and wait for user to end the program.
		printf("Error in Sensor Setup\nPress any key to close the program\n");
		system("pause");
	}
}

int main()
{
	tinySLAM();
	return 1;
}