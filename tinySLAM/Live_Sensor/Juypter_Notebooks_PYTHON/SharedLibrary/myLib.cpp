/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include <unistd.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define TEST_SCAN_SIZE 682
#define TEST_MIN_DIST 20
#define TEST_ANGLE_MIN -180
#define TEST_ANGLE_MAX 180
#define TEST_OFFSET_LASER 145
#define TEST_HOLE_WIDTH 600

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

int BeginSensor(const char * mypath)
{
	const char * opt_com_path = NULL;
	_u32         baudrateArray[2] = { 115200, 256000 };
	_u32         opt_com_baudrate = 0;
	u_result     op_result;
	bool setup = true;

	opt_com_path = mypath;

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
	
	if(setup)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void EndSensor()
{
	drv->stop();
	drv->stopMotor();

	RPlidarDriver::DisposeDriver(drv);
	drv = NULL;
}

void ReadSensor(float map[TEST_SCAN_SIZE])
{
	u_result     op_result;

	// fetech result and print it out...
	rplidar_response_measurement_node_hq_t nodes[8192];
	size_t   count = _countof(nodes);

	//Get the scan
	float vals[TEST_SCAN_SIZE];
	for (int i = 0; i < TEST_SCAN_SIZE; i++)
	{
		//Set Null value equal to Windows Null Value
		vals[i] = -107374176.0;
	}
	
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

int main(float map[TEST_SCAN_SIZE], const char * mypath, int select)
{
	int answer = 0;
	if(select == 0)
	{
		answer = BeginSensor(mypath);
	}
	else if(select == 1)
	{
		ReadSensor(map);
	}
	else
	{
		EndSensor();
	}
	return answer;
}