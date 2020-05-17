#include <iostream>
#include "LiDAR.h"

using namespace std;
#define DLLIMPORT_EXPORT extern "C" __declspec(dllexport)

//Export Sensor Start-up to Python
DLLIMPORT_EXPORT int BeginSensor(const char* port)
{
	//"\\\\.\\com3"
	string vals = port;
	if (StartSensor(vals))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//Export Sensor End to Python
DLLIMPORT_EXPORT void EndSensor()
{
	StopSensor();
}

//Export Read Scan to Python
DLLIMPORT_EXPORT void ReadSensor(float map[682])
{
	read_scan(map);
}



