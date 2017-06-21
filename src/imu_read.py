#! /usr/bin/env python
# Olaya Alvarez Tunon
# Problema 1.1: reading IMU data and computing current position

import rospy
import math
import tf
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from underwater_sensor_msgs.msg import DVL
from geometry_msgs.msg import Pose
# 3D plot
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# IMU data
ax = [0, 0]
ay = [0, 0]
az = [0, 0]

vx = [0, 0]
vy = [0, 0]
vz = [0, 0]


xx = [0, 0]
yy = [0, 0]
zz = [0, 0]

t = [0, 0]

# Real data
x = 0
y = 0
z = 0

# draw 3D plot
fig = plt.figure()
plt.ion()
axis = fig.add_subplot(111, projection='3d')

def int_trapezoidal(x, y, a1):
    a0 = a1+(x[0]-x[1])*(y[1]+y[0])/2
    return a0

def PositionSensor_callback(data, args):
    type = args
    print 'type %s' % (type)
    global ax,ay,az,vx,vy,vz,xx,yy,zz,t
    t[1] = t[0]
    t[0] = rospy.get_time()
    vx[1] = vx[0]
    vy[1] = vy[0]
    vz[1] = vz[0]
    if type == "DVL":
        # if DVL sensor
        vx[0] = data.bi_x_axis
        vy[0] = data.bi_y_axis
        vz[0] = data.bi_z_axis
        print 'DVL'
    elif type == "IMU":
        # if IMU sensor
        ax[1] = ax[0]
        ax[0] = data.linear_acceleration.x
        vx[0] = int_trapezoidal(t, ax, vx[1])
        vy[0] = int_trapezoidal(t, ay, vy[1])
        vz[0] = int_trapezoidal(t, az, vz[1])
        print 'IMU'

    xx[1] = xx[0]
    xx[0] = int_trapezoidal(t, vx, xx[1])

    yy[1] = yy[0]
    yy[0] = int_trapezoidal(t, vy, yy[1])

    zz[1] = zz[0]
    zz[0] = int_trapezoidal(t, vz, zz[1])



def quaternion_mult(q,r):
    mult = [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]
    return mult

def transform_point(point, traslation, quaternion):
    traslated_point = [0]+point #+ traslation
    quaternion_conjugated = [quaternion[0],-1*quaternion[1],-1*quaternion[2],-1*quaternion[3]]
    return quaternion_mult(quaternion_mult(quaternion, traslated_point),quaternion_conjugated)[1:]



def real_pose_callback(data, args):
    global x, y, z
    # Extract the tf transform listener object
    tflisten = args[0]
    # Transform /g500/pose (which is the world frame) to /g500/dvl frame.
    # Time indicates at which time transform. If equal to 0,
    # it gets latest available transform
    (traslation, quaternion) = tflisten.lookupTransform('/girona500/part0','/DVLSensor', rospy.Time(0))

    pose_world_frame = [data.position.x, data.position.y, data.position.z]
    pose_sensor_frame = transform_point(pose_world_frame, traslation, quaternion)
    print '*********************************'
    print quaternion
    print pose_world_frame
    print pose_sensor_frame
    print '*********************************'
    x = pose_sensor_frame[0]
    y = pose_sensor_frame[1]
    z = pose_sensor_frame[2]



class sensor_position:
    def __init__(self,str_topic, msg, tf_listener_object):
        self.str_topic = str_topic
        self.msg = msg
        # Obtains real pose of ROV. Only available in simulations
        rospy.Subscriber("/g500/pose", Pose, real_pose_callback, (tf_listener_object, type))

    def PositionSensor_subscriber(self, type):
        rospy.Subscriber(self.str_topic, self.msg, PositionSensor_callback,(type))




if __name__ == '__main__':
    rospy.init_node('sensor_pose_subscriber', anonymous=True)
    rospy.sleep(10) # when called from roslaunch I need this time to initialize uwsim
    t[1] = rospy.get_time()
    t[0] = rospy.get_time()

    # Create a tf transform listener object
    # Once it is created, it starts receiving tf transformations and buffers for 10 secs
    tflisten = tf.TransformListener()

    spose = sensor_position("/g500/dvl", DVL, tflisten)
    # sensor_position("/g500/imu", Imu)
    spose.PositionSensor_subscriber("DVL")


    while not rospy.is_shutdown():
        axis.scatter(xx[0], yy[0], zz[0], 'o', color='blue')
        axis.scatter(x, y, z, 'o', color='red')

        plt.show()
        plt.pause(0.2)
