Python version of the tinySLAM algorithm using the RPLiDAR

The RPLiDAR - Library folder is based directly off the SDK found at:
http://www.slamtec.com/en/support#rplidar-a-series

The RPLiDAR - Library folder converts the RPLiDAR library into a static library.

The LiDARLib folder handles a method of converting this static library into a dynamic link library (.DLL) for use in the Python code

Finally the tinySLAM.py code runs a version of the tinySLAM algorithm using the RPLiDAR based on the previously made .DLL library.
