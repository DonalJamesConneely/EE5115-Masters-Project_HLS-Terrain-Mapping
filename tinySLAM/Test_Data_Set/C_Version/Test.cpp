#include "windows.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "CoreSLAM.h"

#define TEST_FILENAME "test_lab.dat"
#define TEST_SCAN_SIZE 682
#define TEST_MIN_DIST 20
#define TEST_ANGLE_MIN -120
#define TEST_ANGLE_MAX +120
#define TEST_OFFSET_LASER 145
#define TEST_HOLE_WIDTH 600

using namespace cv;

//Parameter of Robot
typedef struct {
	double r;	    // length wheels' radius
	double R;	    // half the wheels' axis length
	int inc;	    // wheels' counters increments per turn
	double ratio;   // ratio between left and right wheel
} cart_parameters_t;

//Global Variable
ts_sensor_data_t sensor_data[600];
ts_map_t trajectory, map;

//Save Map as a Portable Gray Map (PGM) Image file
void record_map(ts_map_t *map, ts_map_t *overlay, char *filename, int width, int height)
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
			if (overlay->map[(TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 0)
				fprintf(output, "0 ");

			//If value in location. Value equals number/(2^8).
			else
				fprintf(output, "%d ", (int)(map->map[(TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8);
		}
		fprintf(output, "\n");
	}
	fclose(output);
}

//Calculate New Map by updating data points
void draw_scan(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos)
{
	double c, s;
	double x2p, y2p;
	int i, x1, y1, x2, y2;

	c = cos(pos->theta * M_PI / 180);
	s = sin(pos->theta * M_PI / 180);
	x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5);
	y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);

	// Translate and rotate scan to robot position
	for (i = 0; i != scan->nb_points; i++) {
		if (scan->value[i] != TS_NO_OBSTACLE) {
			x2p = c * scan->x[i] - s * scan->y[i];
			y2p = s * scan->x[i] + c * scan->y[i];
			x2p *= TS_MAP_SCALE;
			y2p *= TS_MAP_SCALE;
			x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5);
			y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
			if (x2 >= 0 && y2 >= 0 && x2 < TS_MAP_SIZE && y2 < TS_MAP_SIZE)
				map->map[y2 * TS_MAP_SIZE + x2] = 0;
		}
	}
}

//Use the Monte Carlo Method
ts_position_t monte_carlo_move(ts_scan_t *scan, ts_map_t *map, ts_position_t *start_pos, int debug)
{
	ts_position_t cpp, currentpos, bestpos, lastbestpos;
	int currentdist;
	int bestdist, lastbestdist;
	int counter = 0;

	currentpos = bestpos = lastbestpos = *start_pos;
	currentdist = ts_distance_scan_to_map(scan, map, &currentpos);
	bestdist = lastbestdist = currentdist;

	do {
		currentpos = lastbestpos;
		currentpos.x += 50 * (((double)rand()) / RAND_MAX - 0.5);
		currentpos.y += 50 * (((double)rand()) / RAND_MAX - 0.5);
		currentpos.theta += 50 * (((double)rand()) / RAND_MAX - 0.5);

		currentdist = ts_distance_scan_to_map(scan, map, &currentpos);

		if (currentdist < bestdist) {
			bestdist = currentdist;
			bestpos = currentpos;
			if (debug) printf("Monte carlo ! %lg %lg %lg %d (count = %d)\n", bestpos.x, bestpos.y, bestpos.theta, bestdist, counter);
		}
		else {
			counter++;
		}
		if (counter > 100) {
			if (bestdist < lastbestdist) {
				lastbestpos = bestpos;
				lastbestdist = bestdist;
				counter = 0;
			}
		}
	} while (counter < 1000);
	return bestpos;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// Read Sensor Data //////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int read_sensor_data(ts_sensor_data_t *data)
{
	int i, j, nb_sensor_data = 0;
	int d[TS_SCAN_SIZE];
	ts_scan_t *scan;
	char *str, line[4000];
	double angle_deg, angle_rad;

	FILE *input = fopen(TEST_FILENAME, "rt");
	do {
		// Read the scan
		str = fgets(line, 4000, input);
		if (str == NULL) break;
		str = strtok(str, " ");
		sscanf(str, "%d", &data[nb_sensor_data].timestamp);
		str = strtok(NULL, " ");
		sscanf(str, "%d", &data[nb_sensor_data].q1);
		str = strtok(NULL, " ");
		sscanf(str, "%d", &data[nb_sensor_data].q2);
		data[nb_sensor_data].q2 = -data[nb_sensor_data].q2;
		for (i = 0; i < 10; i++)
			str = strtok(NULL, " ");
		for (i = 0; i < TEST_SCAN_SIZE; i++) {
			if (str) {
				sscanf(str, "%d", &d[i]);
				str = strtok(NULL, " ");
			}
			else d[i] = 0;
		}

		// Change to (x,y) scan
		scan = &data[nb_sensor_data].scan;
		scan->nb_points = 0;
#define SPAN 1
		// Span the laser scans to better cover the space
		for (i = 0; i < TEST_SCAN_SIZE; i++) {
			for (j = 0; j != SPAN; j++) {
				angle_deg = TEST_ANGLE_MIN + ((double)(i * SPAN + j)) * (TEST_ANGLE_MAX - TEST_ANGLE_MIN) / (TEST_SCAN_SIZE * SPAN - 1);
				angle_rad = angle_deg * M_PI / 180;
				if (i > 45 && i < TEST_SCAN_SIZE - 45) {
					if (d[i] == 0) {
						scan->x[scan->nb_points] = TS_DISTANCE_NO_DETECTION * cos(angle_rad);
						scan->y[scan->nb_points] = TS_DISTANCE_NO_DETECTION * sin(angle_rad);
						scan->value[scan->nb_points] = TS_NO_OBSTACLE;
						scan->x[scan->nb_points] += TEST_OFFSET_LASER;
						scan->nb_points++;
					}
					if (d[i] > TEST_HOLE_WIDTH / 2) {
						scan->x[scan->nb_points] = d[i] * cos(angle_rad);
						scan->y[scan->nb_points] = d[i] * sin(angle_rad);
						scan->value[scan->nb_points] = TS_OBSTACLE;
						scan->x[scan->nb_points] += TEST_OFFSET_LASER;
						scan->nb_points++;
					}
				}
			}
		}
		nb_sensor_data++;
	} while (1);

	fclose(input);
	return nb_sensor_data;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////// MAIN ////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
	FILE *output, *output2;
	ts_position_t startpos, position, position2;
	char filename[256];
	int i, x, y, test;
	int nb_sensor_data, cnt_scans;
	int timestamp, told;

	// Read all the scans
	nb_sensor_data = read_sensor_data(sensor_data);
	printf("sensor data = %d\n", nb_sensor_data);

	/////////////////////////////////////// Map Init //////////////////////////////////////////////
	ts_map_init(&map);
	ts_map_init(&trajectory);
	output = fopen("test_trajectory.dat", "wt");

	////////////////////////////////////Initial machine pose//////////////////////////////////////
	position.x = 0.3 * TS_MAP_SIZE / TS_MAP_SCALE;
	position.y = 0.3 * TS_MAP_SIZE / TS_MAP_SCALE;
	position.theta = 0;
	startpos = position2 = position;

	//For all values in the test data
	for (cnt_scans = 0; cnt_scans != nb_sensor_data; cnt_scans++) 
	{
		//Get the Timestamp
		timestamp = sensor_data[cnt_scans].timestamp;

		//Perform Monte-Carlo calculations
		position2 = monte_carlo_move(&sensor_data[cnt_scans].scan, &map, &position, 0);

		/////////////////////////////////Update Map////////////////////////////////////////////
		position = position2;
		printf("#%d : %lg %lg %lg\n", cnt_scans, position.x, position.y, position.theta);
		ts_map_update(&sensor_data[cnt_scans].scan, &map, &position, 50);

		///////////////////////////Set Current Pose as Obstacle Point//////////////////////////
		x = (int)floor(position.x * TS_MAP_SCALE + 0.5);
		y = ((int)floor(position.y * TS_MAP_SCALE + 0.5));

		if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
			trajectory.map[y * TS_MAP_SIZE + x] = 0;

		///////////////////////////Update the obstacle point map///////////////////////////////
		draw_scan(&sensor_data[cnt_scans].scan, &trajectory, &position);

		if (cnt_scans % 100 == 0)
		{
			sprintf(filename, "test_lab%04d.pgm", cnt_scans);
			record_map(&map, &trajectory, filename, TS_MAP_SIZE, TS_MAP_SIZE);

			//Display Map
			Mat img = imread(filename);
			namedWindow("image", WINDOW_NORMAL);
			imshow("image", img);
			waitKey(0);

			circle(img, CvPoint(x, TS_MAP_SIZE-y), 10, (0, 255, 0), 10);
			imshow("image", img);
			waitKey(0);
			destroyWindow("image");
		}

		//Set values equal to current for next loop.
		fprintf(output, "\n");
		told = timestamp;
	}
	fclose(output);

	// Record the map
	sprintf(filename, "test_lab_reversed.pgm");
	record_map(&map, &trajectory, filename, TS_MAP_SIZE, TS_MAP_SIZE);

	//Display Map
	Mat img = imread(filename);
	namedWindow("image", WINDOW_NORMAL);
	imshow("image", img);
	waitKey(0);

	circle(img, CvPoint(x, TS_MAP_SIZE - y), 10, (0, 0, 255), 10);
	imshow("image", img);
	waitKey(0);
	destroyWindow("image");

	return 1;
}
