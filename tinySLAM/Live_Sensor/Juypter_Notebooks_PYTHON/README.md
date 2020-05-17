This Python version of the tinySLAM algorithm using the RPLiDAR on Jupyter Notebooks

# RPLiDAR SDK Setup #
As the PYNQ-Z2 FPGA uses an ARM Linux processor instead of a Windows processor, the RPLiDAR SDK had to be adapted to work using ARM Linux.

The RPLiDAR comes with a Makefile to build the system on standard Linux systems, however, the ARM Linux functions slightly differently and
therefore cannot complete the Library construction.

Therefore the final steps of the Library construction and the adaptaion to PYTHON must be done manually. To do this a the Shared Library
folder was made which contains all the constructed elements of the RPLiDAR library.

Once the Shared Library was gathered, a C wrapper file myLib.cpp was constructed in order to access the required RPLiDAR functions from
PYTHON.

Finally, RPLiDAR PYTHON library was constructed using the following commands in the Jupter Notebooks command terminal:
gcc -Wall -fPIC -c myLib.cpp 
g++ -shared -o libLiDAR.so myLib.o net_serial.o net_socket.o thread.o timer.o rplidar_driver.o -Wl,--no-whole-archive librplidar_sdk.a


# RPLiDAR COM PORT Setup #
In order to allow the PYNQ board access to the RPLiDAR port the PYNQ board perform the following commands:
sudo chmod 666 /dev/ttyUSB0
sudo sysctl kernel.dmesg_restrict=0

These commands must be run everytime the board is powered on before using the RPLiDAR, therefore they can be set to be auto performed on
boot-up using:
sudo crontab -e

Add the following commands to crontab:
@reboot sudo chmod 666 /dev/ttyUSB0
@reboot sudo sysctl kernel.dmesg_restrict=0
