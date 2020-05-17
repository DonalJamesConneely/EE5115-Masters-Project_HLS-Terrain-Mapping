#ifndef _LIDAR_H_
#define _LIDAR_H_

#define TEST_SCAN_SIZE 682
#define TEST_MIN_DIST 20
#define TEST_ANGLE_MIN -180
#define TEST_ANGLE_MAX 180
#define TEST_OFFSET_LASER 145
#define TEST_HOLE_WIDTH 600

using namespace std;

bool StartSensor(string mypath);
void StopSensor();
void read_scan(float map[TEST_SCAN_SIZE]);
#endif