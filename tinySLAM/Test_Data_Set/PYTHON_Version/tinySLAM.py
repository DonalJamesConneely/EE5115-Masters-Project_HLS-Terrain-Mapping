import cv2
import re
import math
from copy import deepcopy
import random
import struct
import time
import numpy as np

class ts_sensor_data_t():
    def __init__(self, TS_SCAN_SIZE):
        self.TS_SCAN_SIZE = TS_SCAN_SIZE
        self.timestamp = 0
        self.q1 = 0
        self.q2 = 0
        self.scan = []

        for i in range(0, self.TS_SCAN_SIZE):
            elements = [0]*4
            self.scan.append(elements)

class ExtendedFunctions():
    def __init__(self):
        self.TEST_SCAN_SIZE = 682
        self.TEST_MIN_DIST  = 20
        self.TEST_ANGLE_MIN = -120
        self.TEST_ANGLE_MAX = +120
        self.TEST_OFFSET_LASER = 145
        self.TEST_HOLE_WIDTH = 600
        self.M_PI = 3.14159265358979323846

        """
        self.TS_SCAN_SIZE = 683
        self.TS_MAP_SIZE = 360
        self.TS_MAP_SCALE = 0.017578125
        self.TS_DISTANCE_NO_DETECTION = 4000
        self.TS_NO_OBSTACLE = 65500
        self.TS_OBSTACLE = 0
        self.TS_HOLE_WIDTH = 600
        """

        self.TS_SCAN_SIZE = 683
        self.TS_MAP_SIZE = 2140
        self.TS_MAP_SCALE = 0.1
        self.TS_DISTANCE_NO_DETECTION = 4000
        self.TS_NO_OBSTACLE = 65500
        self.TS_OBSTACLE = 0
        self.TS_HOLE_WIDTH = 600

        self.val = np.array([1], dtype=np.uint)
        self.RAND_MAX = 0x7fff

    #Copy array from one to the other
    def passValues(self, iscan):
        oscan = iscan[:]
        return oscan

    #Read Data from data file
    def read_sensor_data(self, data):
        nb_sensor_data = 0
        d = [0] * self.TS_SCAN_SIZE
        num = 0

        # Read in data
        input = open('C:/Users/donal/PycharmProjects/tinySLAM_Python/test_lab.dat', 'r')
        while True:
            rstr = input.readline()
            if not rstr:
                break

            str = re.split(" ", rstr)
            data[nb_sensor_data].timestamp = float(str[0])
            data[nb_sensor_data].q1 = float(str[1])
            data[nb_sensor_data].q2 = float(str[2])
            data[nb_sensor_data].q2 = -data[nb_sensor_data].q2

            for i in range(0, self.TEST_SCAN_SIZE):
                try:
                    if (str[i + 18]):
                        d[i] = int(str[i + 18])
                    else:
                        d[i] = 0
                except IndexError:
                    d[i] = 0

            scan = self.passValues(data[nb_sensor_data].scan)
            nb_points = 0
            SPAN = 1
            for i in range(0, self.TEST_SCAN_SIZE):
                for j in range(0, SPAN):
                    angle_deg = self.TEST_ANGLE_MIN + float((i * SPAN + j)) * (self.TEST_ANGLE_MAX - self.TEST_ANGLE_MIN) \
                                / (self.TEST_SCAN_SIZE * SPAN - 1)
                    angle_rad = angle_deg * self.M_PI / 180
                    if ((i > 45) and (i < self.TEST_SCAN_SIZE - 45)):
                        if (d[i] == 0):
                            scan[nb_points][1] = self.TS_DISTANCE_NO_DETECTION * math.cos(angle_rad)
                            scan[nb_points][2] = self.TS_DISTANCE_NO_DETECTION * math.sin(angle_rad)
                            scan[nb_points][3] = self.TS_NO_OBSTACLE
                            scan[nb_points][1] += self.TEST_OFFSET_LASER
                            nb_points += 1
                        if (d[i] > (self.TEST_HOLE_WIDTH / 2)):
                            scan[nb_points][1] = d[i] * math.cos(angle_rad)
                            scan[nb_points][2] = d[i] * math.sin(angle_rad)
                            scan[nb_points][3] = self.TS_OBSTACLE
                            scan[nb_points][1] += self.TEST_OFFSET_LASER
                            nb_points += 1

            for i in range(0, self.TS_SCAN_SIZE):
                scan[i][0] = nb_points

            data[nb_sensor_data].scan = self.passValues(scan)
            nb_sensor_data += 1
            num += 1

        input.close()
        return nb_sensor_data, data


    #Calculate New Map by updating data points
    def draw_scan(self, scan, dmap, dpos):
        c = math.cos(dpos[2] * self.M_PI / 180)
        s = math.sin(dpos[2] * self. M_PI / 180)
        x1 = math.floor(dpos[0] * self.TS_MAP_SCALE + 0.5)
        y1 = math.floor(dpos[1] * self.TS_MAP_SCALE + 0.5)

        for i in range(0, int(scan[0][0])):
            xs = scan[i][1]
            ys = scan[i][2]
            vs = scan[i][3]

            if (vs != self.TS_NO_OBSTACLE):
                x2p = c * xs - s * ys
                y2p = s * xs + c * ys
                x2p *= self.TS_MAP_SCALE
                y2p *= self.TS_MAP_SCALE
                x2 = int(math.floor(dpos[0] * self.TS_MAP_SCALE + x2p + 0.5))
                y2 = int(math.floor(dpos[1] * self.TS_MAP_SCALE + y2p + 0.5))
                if ((x2 >= 0) and (y2 >= 0) and (x2 < self.TS_MAP_SIZE) and (y2 < self.TS_MAP_SIZE)):
                    dmap[y2 * self.TS_MAP_SIZE + x2] = 0


    def record_map(self, imap, overlay, filename, width, height):
        output2 = open(filename, 'w')

        output2.write('P2\n%d %d 255\n' % (width, height))
        ry = (self.TS_MAP_SIZE - height) / 2

        for yp in range(0, height):
            rx = (self.TS_MAP_SIZE - width) / 2
            for xp in range(0, width):
                if (overlay[int((self.TS_MAP_SIZE - 1 - ry) * self.TS_MAP_SIZE + rx)] == 0):
                    output2.write('0 ')
                else:
                    output2.write("%d " % (int((imap[int((self.TS_MAP_SIZE - 1 - ry) * self.TS_MAP_SIZE + rx)]) >> 8)))

                rx += 1
            output2.write('\n')
            ry += 1

        output2.close()

class Core_Functions():
    def __init__(self):
        self.TEST_SCAN_SIZE = 682
        self.TEST_MIN_DIST  = 20
        self.TEST_ANGLE_MIN = -120
        self.TEST_ANGLE_MAX = +120
        self.TEST_OFFSET_LASER = 145
        self.TEST_HOLE_WIDTH = 600
        self.M_PI = 3.14159265358979323846

        self.TS_SCAN_SIZE = 683
        self.TS_MAP_SIZE = 2140
        self.TS_MAP_SCALE = 0.1
        self.TS_DISTANCE_NO_DETECTION = 4000
        self.TS_NO_OBSTACLE = 65500
        self.TS_OBSTACLE = 0
        self.TS_HOLE_WIDTH = 600

        self.val = np.array([1], dtype=np.uint)
        self.RAND_MAX = 0x7fff


    # //////////////////////////////////////////////////// Random ////////////////////////////////////////////////
    def myrand(self):
        self.val = (self.val * 1103515245) + 12345
        answer = int((self.val / 65536) % self.RAND_MAX)
        return answer

    # //////////////////////////////////////////////////// MAP INIT /////////////////////////////////////////////////////
    def Map_Init(self):
        imap = [0]*(self.TS_MAP_SIZE * self.TS_MAP_SIZE)
        initval = (self.TS_OBSTACLE + self.TS_NO_OBSTACLE)/2

        for i in range(0, (self.TS_MAP_SIZE * self.TS_MAP_SIZE)):
            imap[i] = initval

        return imap

    # /////////////////////////////////////////////////// MONTE CARLO //////////////////////////////////////////////////////
    def tiny_distance(self, scan, mp, pos):
        nb_points = 0
        posx = pos[0]
        posy = pos[1]
        postheta = pos[2]
        sum_out = 0

        c = float(math.cos(postheta * self.M_PI / 180))
        s = float(math.sin(postheta * self.M_PI / 180))

        #//Translate and rotate scan to robot position and compute the distance.
        for i in range(0, (int(scan[0][0]))):
            scanx = scan[i][1]
            scany = scan[i][2]
            scanvalue = (int(scan[i][3]))

            if(scanvalue != self.TS_NO_OBSTACLE):
                dx = (int(math.floor((posx + c * scanx - s * scany) * self.TS_MAP_SCALE + 0.5)))
                dy = (int(math.floor((posy + s * scanx + c * scany) * self.TS_MAP_SCALE + 0.5)))

                if((dx >= 0) and (dx < self.TS_MAP_SIZE) and (dy >= 0) and (dy < self.TS_MAP_SIZE)):
                    sum_out += mp[dy * self.TS_MAP_SIZE + dx]
                    nb_points += 1

        if (nb_points):
            sum_out = sum_out * 1024 / nb_points
        else:
            sum_out = 2000000000

        return(int(sum_out))

    def Monte_Carlo(self, scan, mp, pos):
        counter = 0
        curpos = pos[:]
        bestpos = pos[:]
        lastbestpos = pos[:]
        currentdist = core_func.tiny_distance(scan, mp, curpos)
        bestdist = currentdist
        lastbestdist = currentdist

        while (counter < 1000):
            for i in range(0, 3):
                curpos[i] = lastbestpos[i]

            curpos[0] += 50 * (float(self.myrand()) / self.RAND_MAX - 0.5)
            curpos[1] += 50 * (float(self.myrand()) / self.RAND_MAX - 0.5)
            curpos[2] += 50 * (float(self.myrand()) / self.RAND_MAX - 0.5)

            currentdist = core_func.tiny_distance(scan, mp, curpos)

            if (currentdist < bestdist):
                bestdist = currentdist
                for i in range(0, 3):
                    bestpos[i] = curpos[i]
            else:
                counter += 1

            if (counter > 100):
                if (bestdist < lastbestdist):
                    for i in range(0, 3):
                        lastbestpos[i] = bestpos[i]
                    lastbestdist = bestdist
                    counter = 0

        return bestpos

    # ///////////////////////////////////////////////////// MAP UPDATE //////////////////////////////////////////////////
    def SWAP(self, a, b):
        return b, a

    def Map_laser_ray(self, mp, x1, y1, x2, y2, xp, yp, value, alpha):
        if ((x1 < 0) or (x1 >= self.TS_MAP_SIZE) or (y1 < 0) or (y1 >= self.TS_MAP_SIZE)):
            return

        x2c = x2
        y2c = y2

        #//Clipping
        if (x2c < 0):
            if (x1 == x2c):
                return
            y2c += int((y2c - y1) * (float(-x2c) / (x2c - x1)))
            x2c = 0

        if (x2c >= self.TS_MAP_SIZE):
            if (x1 == x2c):
                return
            y2c += int((y2c - y1) * (float(self.TS_MAP_SIZE - 1 - x2c) / (x2c - x1)))
            x2c = self.TS_MAP_SIZE - 1

        if (y2c < 0):
            if (y1 == y2c):
                return
            x2c += int((x1 - x2c) * (float(-y2c) / (y1 - y2c)))
            y2c = 0

        if (y2c >= self.TS_MAP_SIZE):
            if (y1 == y2c):
                return
            x2c += int((x1 - x2c) * (float(self.TS_MAP_SIZE - 1 - y2c) / (y1 - y2c)))
            y2c = self.TS_MAP_SIZE - 1

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        dxc = abs(x2c - x1)
        dyc = abs(y2c - y1)
        incptrx = -1
        incptry = -self.TS_MAP_SIZE
        sincv = -1

        if(x2 > x1):
            incptrx = 1

        if (y2 > y1):
            incptry = self.TS_MAP_SIZE

        if(value > self.TS_NO_OBSTACLE):
            sincv = 1

        if (dx > dy):
            derrorv = abs(xp - x2)
        else:
            dx, dy = self.SWAP(dx, dy)
            dxc, dyc = self.SWAP(dxc, dyc)
            incptrx, incptry = self.SWAP(incptrx, incptry)
            derrorv = abs(yp - y2)

        error = 2 * dyc - dxc
        horiz = 2 * dyc
        diago = 2 * (dyc - dxc)
        errorv = derrorv / 2
        incv = (value - self.TS_NO_OBSTACLE) / derrorv
        incerrorv = value - self.TS_NO_OBSTACLE - derrorv * incv
        ptr = y1 * self.TS_MAP_SIZE + x1
        pixval = self.TS_NO_OBSTACLE

        for x in range(0, dxc+1):
            if (x > dx - 2 * derrorv):
                if (x <= dx - derrorv):
                    pixval += incv
                    errorv += incerrorv
                    if (errorv > derrorv):
                        pixval += sincv
                        errorv -= derrorv
                else:
                    pixval -= incv
                    errorv -= incerrorv
                    if (errorv < 0):
                        pixval -= sincv
                        errorv += derrorv

            mp[ptr] = ((256 - alpha) * (mp[ptr]) + alpha * pixval) >> 8
            if (error > 0):
                ptr += incptry
                error += diago
            else:
                error += horiz

            ptr += incptrx

    def Map_Update(self, scan, mp, ipos, quality):
        posx = ipos[0]
        posy = ipos[1]
        postheta = ipos[2]

        c = float(math.cos(postheta * self.M_PI / 180))
        s = float(math.sin(postheta * self.M_PI / 180))

        x1 = (int(math.floor(posx * self.TS_MAP_SCALE + 0.5)))
        y1 = (int(math.floor(posy * self.TS_MAP_SCALE + 0.5)))

        #//Translate and rotate can to robot position
        for i in range(0, (int(scan[0][0]))):
            scanx = scan[i][1]
            scany = scan[i][2]
            scanvalue = int(scan[i][3])

            x2p = c * scanx - s * scany
            y2p = s * scanx + c * scany

            xp = (int(math.floor((posx + x2p) * self.TS_MAP_SCALE + 0.5)))
            yp = (int(math.floor((posy + y2p) * self.TS_MAP_SCALE + 0.5)))
            dist = math.sqrt(x2p * x2p + y2p * y2p)

            add = self.TS_HOLE_WIDTH / 2 / dist
            x2p *= self.TS_MAP_SCALE * (1 + add)
            y2p *= self.TS_MAP_SCALE * (1 + add)

            x2 = (int(math.floor(posx * self.TS_MAP_SCALE + x2p + 0.5)))
            y2 = (int(math.floor(posy * self.TS_MAP_SCALE + y2p + 0.5)))

            if (scanvalue == self.TS_NO_OBSTACLE):
                q = quality / 2
                value = self.TS_NO_OBSTACLE
            else:
                q = quality
                value = self.TS_OBSTACLE

            self.Map_laser_ray(mp, x1, y1, x2, y2, xp, yp, value, q)

        return mp

################################################## MAIN ################################################################
# //Initialise Variables
functions = ExtendedFunctions()
core_func = Core_Functions()
Sensor_Data = []
told = 0
filename = ""

# Initialize Arrays
scaner = []
pos = [0]*3

for i in range(0, 600):
    Sensor_Data.append(ts_sensor_data_t(functions.TS_SCAN_SIZE))

for i in range(0, functions.TS_SCAN_SIZE):
    elements = [0] * 4
    scaner.append(elements)

# // Read all the scans
print("Reading Sensor Data...")
nb_sens_dat, Sensor_Data = functions.read_sensor_data(Sensor_Data)
print(nb_sens_dat)

# ////////////////////////////////////Initial machine pose//////////////////////////////////////
pos[0] = (functions.TS_MAP_SIZE / functions.TS_MAP_SCALE) * 0.3
pos[1] = (functions.TS_MAP_SIZE / functions.TS_MAP_SCALE) * 0.3
pos[2] = 0.0

print("Map Init.....")
mp = core_func.Map_Init()
traj = core_func.Map_Init()

output = open("test_trajectory.dat", 'w')

for cnt_scans in range(0, nb_sens_dat):
    for i in range(0, functions.TS_SCAN_SIZE):
        scaner[i][0] = Sensor_Data[cnt_scans].scan[i][0]
        scaner[i][1] = Sensor_Data[cnt_scans].scan[i][1]
        scaner[i][2] = Sensor_Data[cnt_scans].scan[i][2]
        scaner[i][3] = Sensor_Data[cnt_scans].scan[i][3]

    timestamp = Sensor_Data[cnt_scans].timestamp

    # //Perform Monte-Carlo calculations
    # print("Monte Carlo....")
    pos2 = core_func.Monte_Carlo(scaner, mp, pos)

    # /////////////////////////////////Update Map////////////////////////////////////////////
    for i in range(0, 3):
        pos[i] = pos2[i]

    print("#%d : %lg %lg %lg" % (cnt_scans, pos[0], pos[1], pos[2]))

    # print("Map Update.....")
    mp = core_func.Map_Update(scaner, mp, pos, 50)

    # ///////////////////////////Set Current Pose as Obstacle Point//////////////////////////
    x = (int(math.floor(pos[0] * functions.TS_MAP_SCALE + 0.5)))
    y = (int(math.floor(pos[1] * functions.TS_MAP_SCALE + 0.5)))

    if ((x >= 0) and (y >= 0) and (x < functions.TS_MAP_SIZE) and (y < functions.TS_MAP_SIZE)):
        traj[y * functions.TS_MAP_SIZE + x] = 0

    # ///////////////////////////Update the obstacle point map///////////////////////////////
    functions.draw_scan(scaner, traj, pos)

    # //Set values equal to current for next loop.
    told = timestamp
    output.write("\n")

output.close()

# // Record the map
print("Converting to PGM")
filename = "C:/Users/donal/PycharmProjects/tinySLAM_Python/test_lab_reversed.pgm"
functions.record_map(mp, traj, filename, functions.TS_MAP_SIZE, functions.TS_MAP_SIZE)

img = cv2.imread(filename)
cv2.namedWindow("image", cv2.WINDOW_NORMAL)
cv2.imshow("image", img)
cv2.waitKey()
print("Finished!!")
