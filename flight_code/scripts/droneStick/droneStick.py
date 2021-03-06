#!/usr/bin/env python

import numpy as np
import time
import swarmlib
from swarmlib import Drone
import rospy
import math

import crazyflie
from crazyflie_driver.msg import FullState
from crazyflie_driver.msg import Position

import matplotlib.pyplot as plt
from potential_fields import *
from tools import *
import os
from multiprocessing import Process

def publish_goal_pos(cf_goal_pos, cf_goal_yaw, cf_name):
    name = cf_name + "/cmd_position"
    msg = msg_def_crazyflie(cf_goal_pos, cf_goal_yaw)
    pub = rospy.Publisher(name, Position, queue_size=1)
    pub.publish(msg)

def msg_def_crazyflie(pose, yaw):
    worldFrame = rospy.get_param("~worldFrame", "/world")
    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.x = pose[0]
    msg.y = pose[1]
    msg.z = pose[2]
    msg.yaw = yaw
    now = rospy.get_time()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    return msg

def hover(t=5):
    print "Hovering...\n"
    while not rospy.is_shutdown():
        for i in range(int(t*100 / len(drones))):
            for drone in drones:
                if toFly: drone.fly()
                drone.publish_sp()
                time.sleep(0.01)
        break

def landing():
    print 'Landing!!!'
    for drone in drones: drone.sp = drone.position()
    while(1):
        for drone in drones:
            drone.sp[2] = drone.sp[2]-0.05
            drone.fly()
            time.sleep(0.1)

        if drones[0].sp[2]<-0.5:
            print 'reached the floor'
            if data_recording: os.system("rosnode kill recorder")
            time.sleep(0.1)
            if toFly:
                for t in range(3):
                    for cf in cf_list: cf.stop()
            break


def start_recording(cf_names, obstacles_names, folder_name='recorded_data'):
    PATH = '~/Desktop/'
    topics = ''
    for cf_name in cf_names:
        topics += '/vicon/'+cf_name+'/'+cf_name   +   ' /'+cf_name+'_sp '
    for name in obstacles_names:
        topics += '/vicon/'+name+'/'+name + ' '
    os.system("mkdir -p "+PATH+folder_name)
    node_name = 'recorder'
    os.system("rosbag record -o "+PATH+folder_name+'/data ' + topics +' __name:='+node_name)



class Robot(Drone):
    def __init__(self, name):
        Drone.__init__(self, name)
        self.route = np.array([self.position()])
        self.f = 0
        self.leader = False


    def local_planner(self, obstacles_poses, params):
        # self.f = combined_potential(obstacles_poses, params.R_obstacles, self.sp[:2], params.obstacles_influence_radius)
        self.f = combined_potential(obstacles_poses, self.sp[:2], params.obstacles_influence_radius)
        self.sp[:2] = gradient_planner(self.sp[:2], self.f)
        # self.route = np.vstack( [self.route, self.sp] )

class Params:
    def __init__(self,):
        self.R_obstacles = 0.10 # [m]
        self.obstacles_influence_radius = 0.1 # [m]
        self.l = 0.4 # [m], inter-drones distance

rospy.init_node('CrazyflieAPI', anonymous=False)

toFly          = 1
TAKEOFFHEIGHT  = 0.8
TAKEOFFTIME    = 3.0
LANDTIME       = 2.0
JOYSTICK_TAKEOFF_HEIGHT = 0.8

initialized    = False
vel_koef       = 3.0
yaw_koef       = 3.5
put_limits       = 1
# limits           = np.array([ 2.0, 2.0, 2.5 ]) # limits desining safety flight area in the room
# limits_negative  = np.array([ -2.0, -2.0, -0.1 ])
limits           = np.array([ 1.7, 1.7, 2.5 ]) # limits desining safety flight area in the room
limits_negative  = np.array([-1.7, -1.7, -0.1 ])
repel_robots   = 1
keep_formation = 0
collision_avoidance = 1
data_recording = 0

params = Params()

# joystick
drone_joystick_name = 'cf3' # 'cf3'
drone_joystick = Drone(drone_joystick_name)

# drones-followers
# cf_names = ['cf1', 'cf2', 'cf3']
# cf_names = ['cf1', 'cf2']
cf_names = ['cf2']


drones = []
drones_poses = []
for name in cf_names:
    drone = Robot(name)
    drones.append( drone )
    drones_poses.append(drone.position())

obstacles = []
obstacles_poses = []
# obstacles_names = []
# obstacles_names = ['obstacle4', 'obstacle10', 'obstacle12', 'obstacle25']
# obstacles_names = ['obstacle25', 'obstacle10', 'obstacle12',]#, 'obstacle4']
# obstacles_names = ['obstacle10', 'obstacle13', 'obstacle15']
obstacles_names = ['obstacle13']
for name in obstacles_names:
    obstacle = swarmlib.Obstacle(name)
    obstacles.append( obstacle )
    obstacles_poses.append(obstacle.position()[:2])
    

if __name__ == "__main__":
    if data_recording:
        print "Data recording started"
        pose_recording = Process(target=start_recording, args=(cf_names + [drone_joystick_name], obstacles_names,))
        pose_recording.start()

    if toFly:
        cf_list = []
        for cf_name in cf_names:
            # print "adding.. ", cf_name
            cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
            cf.setParam("commander/enHighLevel", 1)
            cf.setParam("stabilizer/estimator",  2) # Use EKF
            cf.setParam("stabilizer/controller", 2) # Use mellinger controller
            cf_list.append(cf)
        for t in range(3):
            print "takeoff.. ", cf.prefix
            for cf in cf_list:
                cf.takeoff(targetHeight = TAKEOFFHEIGHT, duration = TAKEOFFTIME)
        time.sleep(TAKEOFFTIME+2.0)
        # go to approximate human location
        # for t in range(3):
        # for cf in cf_list:
        #     cf.goTo(goal = [0.5, 0.5, 0], yaw=0.0, duration=3.0, relative = True)
        # time.sleep(3.5)

        # time.sleep(100)
        # for t in range(3):
        #     for cf in cf_list: cf.land(targetHeight = -0.05, duration = 3.0)

     # setpoint estimation
    print 'Joystick mean orientation estimation...\n' 
    time_to_eval = 0.8
    a = drone_joystick.orientation()
    for i in range(int(time_to_eval*100)):
        a = np.vstack([a, drone_joystick.orientation()])
        time.sleep(0.01)
    mean_angles = np.array([np.mean(a[:,0]), np.mean(a[:,1]), np.mean(a[:,2])])

    print 'start DroneStick \n'
    # time_to_play = 300
    # for i in range(time_to_play*100):
    plt.figure(figsize=(8,8))
    while not rospy.is_shutdown():
        drone_joystick.orient = drone_joystick.orientation()
        drone_joystick.position()
        for drone in drones: drone.pose = drone.position()
        roll = drone_joystick.orient[0] - mean_angles[0]
        pitch = drone_joystick.orient[1] - mean_angles[1]
        yaw = drone_joystick.orient[2] - mean_angles[2]
        dz = drone_joystick.pose[2] - JOYSTICK_TAKEOFF_HEIGHT

        # update obstacles poses
        for i in range(len(obstacles)):
            obstacles_poses[i] = obstacles[i].position()[:2]

        # update drones poses
        for i in range(len(drones)):
            drones_poses[i] = drones[i].sp[:2]

        if not initialized:
            for drone in drones: drone.sp = np.array( [drone.pose[0], drone.pose[1], TAKEOFFHEIGHT] )
            time_prev = time.time()
            initialized = True

        pitch_thresh = [0.05, 0.30]
        roll_thresh = [0.05, 0.30]
        yaw_thresh = [0.05, 0.50]
        if abs(pitch)<pitch_thresh[0]:
            x_input = 0
        elif abs(pitch)>pitch_thresh[1]:
            x_input = 0#- np.sign(pitch) * pitch_thresh[1]
        else:
            x_input = - pitch

        if abs(roll)<roll_thresh[0]:
            y_input = 0
        elif abs(roll)>roll_thresh[1]:
            y_input = 0#np.sign(roll) * roll_thresh[1]
        else:
            y_input = roll

        if abs(yaw)<yaw_thresh[0]:
            yaw_input = 0
        elif abs(yaw)>roll_thresh[1]:
            yaw_input = np.sign(yaw) * yaw_thresh[1]
        else:
            yaw_input = yaw

        dz_thresh = [0.04, 0.15]
        if abs(dz)<dz_thresh[0]:
            z_input = 0
        elif abs(dz)>dz_thresh[1]:
            z_input = np.sign(dz) * dz_thresh[1]
        else:
            z_input = dz

        cmd_vel = vel_koef*(np.array([x_input, y_input, z_input]))
        yaw_input = yaw_koef * yaw_input
        # print 'cmd_vel', cmd_vel
        # np.putmask(cmd_vel, abs(cmd_vel) <= (vel_koef*0.035), 0)
        # np.putmask(cmd_vel, abs(cmd_vel) <= (0.20), 0)
        time_now = time.time()
        for drone in drones: drone.sp += cmd_vel*(time.time()-time_prev)
        time_prev = time_now


        # drones formation:
        if keep_formation and len(drones)>1:
            # drones[1].sp = drones[0].sp + np.array([-0.86*params.l , params.l/2., 0])
            drones[1].sp = drones[0].sp + np.array([0 , -params.l, 0])
            # drones[2].sp = drones[0].sp + np.array([-0.86*params.l ,-params.l/2., 0])

        # correct point to follow with local planner
        if collision_avoidance:
            for p in range(len(drones)):
                if repel_robots:
                    robots_obstacles = [x for i,x in enumerate(drones_poses) if i!=p] # all poses except the robot[p]
                    obstacles_poses1 = obstacles_poses + robots_obstacles
                    drones[p].local_planner(obstacles_poses1, params)
                else:
                    drones[p].local_planner(obstacles_poses, params)

        if put_limits:
            for drone in drones:
                np.putmask(drone.sp, drone.sp >= limits, limits)
                np.putmask(drone.sp, drone.sp <= limits_negative, limits_negative)

        if toFly:
            for drone in drones: drone.fly_yaw(yaw=yaw_input)

        # visualization: RVIZ
        for drone in drones:
            drone.publish_sp(orient=np.array([0,0,yaw_input]))
            drone.publish_path_sp()
        for obstacle in obstacles: obstacle.publish_position()
        # visualization: matplotlib
        plt.cla()
        if collision_avoidance: draw_gradient(drones[0].f)
        draw_map(obstacles_poses, params.R_obstacles)
        for drone in drones: plt.plot(drone.sp[0], drone.sp[1], '^', markersize=10, label=drone.name)
        for drone in drones:
            drone.route = np.vstack( [drone.route, drone.sp] )
            plt.plot(drone.route[:,0], drone.route[:,1], '-', color='green', linewidth=3, label=drone.name+'_route')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.draw()
        plt.pause(0.01)


        # RETURN JOYSTICK TO THE SWARM
        Z = JOYSTICK_TAKEOFF_HEIGHT
        # if drone_joystick.pose[2] > Z + 0.07:
        if (Z - drone_joystick.pose[2]) > 0.15 and toFly:
            # print 'Returning joystick...'
            # hover(t=4)

            landing()

            break


