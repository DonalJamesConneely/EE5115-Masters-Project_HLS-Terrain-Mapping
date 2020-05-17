//#include "windows.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rplidar.h"
#include "LiDAR.h"
#include <signal.h>

using namespace std;
using namespace rp::standalone::rplidar;

//Global Variable
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

bool StartSensor(string mypath)
{
	const char * opt_com_path = NULL;
	_u32         baudrateArray[2] = { 115200, 256000 };
	_u32         opt_com_baudrate = 0;
	u_result     op_result;
	bool setup = true;

	opt_com_path = mypath.c_str();

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

void read_scan(float map[TEST_SCAN_SIZE])
{
	u_result     op_result;

	// fetech result and print it out...
	rplidar_response_measurement_node_hq_t nodes[8192];
	size_t   count = _countof(nodes);

	//Get the scan
	float vals[TEST_SCAN_SIZE];
	op_result = drv->grabScanDataHq(nodes, count);

	if (IS_OK(op_result))
	{
		drv->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count; ++pos)
		{
			vals[pos] = (nodes[pos].dist_mm_q2 / 4.0f);
		}
	}

	for (int i = 0; i < TEST_SCAN_SIZE; i++)
	{
		map[i] = vals[i];
	}
}