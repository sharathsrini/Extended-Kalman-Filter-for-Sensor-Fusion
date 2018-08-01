import numpy as np
import matplotlib.pyplot as plt
import vehicle
import planner
from scipy import interpolate
import vehicle_lib
import math
import scipy.spatial

def show_animation(path, oox, ooy, other_x=None, other_y=None, other_yaw0=None):
    x = path.x
    y = path.y
    yaw = path.yaw
    direction = path.direction
    steer = 0.0
    for ii in range(0,len(x),20):
        plt.cla()
        plt.plot(oox, ooy, ".k")
        plt.scatter(x, y)

        if ii < len(x)-1:
            k = (yaw[ii+1] - yaw[ii])/planner.MOTION_RESOLUTION
            if direction[ii] == False:
                k *= -1
            steer = math.atan2(vehicle_lib.WB*k, 1.0)
        else:
            steer = 0.0

        if other_x != None and other_y != None and other_yaw0 != None:
            vehicle.plot_trailer(other_x, other_y, other_yaw0, 0)
        vehicle.plot_trailer(x[ii], y[ii], yaw[ii], steer)
        plt.grid(True)
        plt.axis("equal")
        plt.pause(0.01)


SX = 10.0  # [m]
SY = 10.0  # [m]
SYAW = np.deg2rad(0.0)

# goal state
GX = 50  # [m]
GY = 50  # [m]
GYAW = np.deg2rad(90)

ox = []
oy = []
for i in range(60):
	ox.append(i)
	oy.append(0.0) 
for i in range(60):
    ox.append(60.0)
    oy.append(i)
for i in range(61):
    ox.append(i)
    oy.append(60.0)
for i in range(61):
    ox.append(0.0)
    oy.append(i)
for i in range(20):
	ox.append(i)
	oy.append(30.0)
for i in range(45):
	ox.append(i)
	oy.append(40.0)


plt.grid(True)
plt.axis("equal")
plt.plot(ox,oy,'.k')
vehicle.plot_trailer(SX, SY, SYAW, 0)



path1 = planner.calc_hybrid_astar_path(SX, SY, SYAW, GX, GY,GYAW, ox, oy,
                                            planner.XY_GRID_RESOLUTION,
                                            planner.YAW_GRID_RESOLUTION)
#plt.scatter(path1.x, path1.y, "-r", label="Hybrid A* path")
show_animation(path1, ox, oy)
# #plt.scatter(x_path,y_path)
plt.axis([-40, 40, -40, 40])
plt.grid(True)
plt.axis("equal")
# plt.plot(ox, oy, '.k')


plt.show()


