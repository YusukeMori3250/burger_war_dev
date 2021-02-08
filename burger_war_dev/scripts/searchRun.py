#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import rosparam
import math
import copy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs


class TPos:
    def __init__(self,x,y,rot):
        self.x = x
        self.y = y
        self.rot = rot


PLAYER_MARKERS = ["BL_B","BL_L","BL_R","RE_B","RE_L","RE_R"]
FIELD_MARKERS = ["Tomato_N","Tomato_S", "Omelette_N","Omelette_S",
    "Pudding_N","Pudding_S", "OctopusWiener_N","OctopusWiener_S",
    "FriedShrimp_N","FriedShrimp_E","FriedShrimp_W","FriedShrimp_S"]

MARKER_TARGETS = {"Tomato_N":TPos(0.9,0.5,math.pi),"Tomato_S":TPos(0.0,0.5,0.0), "Omelette_N":TPos(0.9,-0.5,math.pi),"Omelette_S":TPos(0.0,-0.5,0.0),
    "Pudding_N":TPos(0.0,0.5,math.pi),"Pudding_S":TPos(-0.9,0.5,0.0), "OctopusWiener_N":TPos(0.0,-0.5,math.pi),"OctopusWiener_S":TPos(-0.9,-0.5,0.0),
    "FriedShrimp_N":TPos(0.5,0.0,math.pi),"FriedShrimp_E":TPos(0.0,-0.5,math.pi/2),"FriedShrimp_W":TPos(0.0,0.5,-math.pi/2),"FriedShrimp_S":TPos(-0.5,0.0,0.0)}


##== from sample_program level_3_clubhouse.py ==##
# respect is_point_enemy from team rabbit
# https://github.com/TeamRabbit/burger_war
class EnemyDetector:
    '''
    Lidarのセンサ値から簡易的に敵を探す。
    obstacle detector などを使ったほうがROSらしいがそれは参加者に任せます。
    いろいろ見た感じ Team Rabit の実装は綺麗でした。
    方針
    実測のLidarセンサ値 と マップと自己位置から期待されるLidarセンサ値 を比較
    ズレている部分が敵と判断する。
    この判断は自己位置が更新された次のライダーのコールバックで行う。（フラグで管理）
    0.7m 以上遠いところは無視する。少々のズレは許容する。
    '''
    def __init__(self):
        self.max_distance = 0.7
        self.thresh_corner = 0.25
        self.thresh_center = 0.35

        self.pose_x = 0
        self.pose_y = 0
        self.th = 0


    def findEnemy(self, scan, pose_x, pose_y, th):
        '''
        input scan. list of lider range, robot locate(pose_x, pose_y, th)
        return is_near_enemy(BOOL), enemy_direction[rad](float)
        '''
        if not len(scan) == 360:
            return False

        # update pose
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.th = th

        # drop too big and small value ex) 0.0 , 2.0
        near_scan = [x if self.max_distance > x > 0.1 else 0.0 for x in scan]

        enemy_scan = [1 if self.is_point_emnemy(x,i) else 0 for i,x in  enumerate(near_scan)]

        is_near_enemy = sum(enemy_scan) > 5  # if less than 5 points, maybe noise
        if is_near_enemy:
            idx_l = [i for i, x in enumerate(enemy_scan) if x == 1]
            idx = idx_l[len(idx_l)/2]
            enemy_direction = idx / 360.0 * 2*PI
            enemy_dist = near_scan[idx]
        else:
            enemy_direction = None
            enemy_dist = None

        print("Enemy: {}, Direction: {}".format(is_near_enemy, enemy_direction))
        print("enemy points {}".format(sum(enemy_scan)))
        return is_near_enemy, enemy_direction, enemy_dist


    def is_point_emnemy(self, dist, ang_deg):
        if dist == 0:
            return False

        ang_rad = ang_deg /360. * 2 * PI
        point_x = self.pose_x + dist * math.cos(self.th + ang_rad)
        point_y = self.pose_y + dist * math.sin(self.th + ang_rad)

        #フィールド内かチェック
        if   point_y > (-point_x + 1.53):
            return False
        elif point_y < (-point_x - 1.53):
            return False
        elif point_y > ( point_x + 1.53):
            return False
        elif point_y < ( point_x - 1.53):
            return False

        #フィールド内の物体でないかチェック
        len_p1 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y - 0.53), 2))
        len_p2 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y + 0.53), 2))
        len_p3 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y - 0.53), 2))
        len_p4 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y + 0.53), 2))
        len_p5 = math.sqrt(pow(point_x         , 2) + pow(point_y         , 2))

        if len_p1 < self.thresh_corner or len_p2 < self.thresh_corner or len_p3 < self.thresh_corner or len_p4 < self.thresh_corner or len_p5 < self.thresh_center:
            return False
        else:
            #print(point_x, point_y, self.pose_x, self.pose_y, self.th, dist, ang_deg, ang_rad)
            #print(len_p1, len_p2, len_p3, len_p4, len_p5)
            return True
# End Respect
##== from sample_program level_3_clubhouse.py ==##


class TsukimiBurger():
    def __init__(self, bot_name):
        # bot name
        self.name = bot_name
        self.side_color = rosparam.get_param("/searchRun/side")
        # robot state 'go' or 'back'
        self.state = 'back'
        # robot wheel rot
        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.pose_x = 0
        self.pose_y = 0

        # speed [m/s]
        self.speed = 0.12

        self.target_markers = []

        # publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # subscriber
        self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        self.war_state_sub = rospy.Subscriber('war_state', String, self.warstateCallback)


    def jointstateCallback(self, data):
        '''
        update wheel rotation num
        '''
        # find left and right wheel_state index
        r_joint_idx = data.name.index("wheel_right_joint")
        l_joint_idx = data.name.index("wheel_left_joint")

        # update joint state value
        self.wheel_rot_r = data.position[r_joint_idx]
        self.wheel_rot_l = data.position[l_joint_idx]

    def poseCallback(self, data):
        '''
        pose topic from amcl localizer
        update robot twist
        '''
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        th = rpy[2]


    def warstateCallback(self,data):
        state = json.loads(data.data)
        tmp = []
        print("color: " + self.side_color)
        for j in state["targets"]:
            if j["name"] in PLAYER_MARKERS:
                continue
            if j["player"] != self.side_color:
                tmp.append(j["name"])
        self.target_markers = tmp

        # goal = MARKER_TARGETS[self.calcNearTarget()]
        # self.setGoal(goal.x,goal.y,goal.rot)

    def calcNearTarget(self):
        index = 0
        min_dist = 100.0
        for t in range(len(self.target_markers)):
            tpos = MARKER_TARGETS[self.target_markers[t]]
            dist = ( (self.pose_x - tpos.x)**2 + (self.pose_y - tpos.y)**2 ) **0.5
            if dist < min_dist:
                index = t
                min_dist = dist
            print(self.pose_x,self.pose_y, tpos.x , tpos.y)
        print(self.target_markers[index])
        return self.target_markers[index]

    def sortTargetMarkers(self):
        res = []
        while len(self.target_markers) != 0:
            t = self.calcNearTarget()
            res.append(t)
            self.target_markers.remove(t)
        self.target_markers = res
        print(self.target_markers)

    def calcTwist(self):
        '''
        calc twist from self.state
        'go' -> self.speed,  'back' -> -self.speed
        '''
        if self.state == 'go':
            # set speed x axis
            x = self.speed
        elif self.state == 'back':
            # set speed x axis
            x = -1 * self.speed
        else:
            # error state
            x = 0
            rospy.logerr("SioBot state is invalid value %s", self.state)

        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        return twist

    def calcState(self):
        '''
        update robot state 'go' or 'back'
        '''
        if self.state == 'go' and self.wheel_rot_r > 28:
            self.state = 'back'
        elif self.state == 'back' and self.wheel_rot_r < 5:
            self.state = 'go'

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()
        print(self.client.get_state())

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def setRoute(self):
        for marker in self.target_markers:
            self.setGoal(MARKER_TARGETS[marker].x,MARKER_TARGETS[marker].y,MARKER_TARGETS[marker].rot)

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        Go and Back loop forever
        '''
        r = rospy.Rate(5) # change speed 1fps

        while not rospy.is_shutdown():
            self.sortTargetMarkers()
            self.setRoute()
            # self.calcState()
            # twist = self.calcTwist()
            # self.vel_pub.publish(twist)



            r.sleep()


if __name__ == '__main__':
    rospy.init_node('Otsukimi_run')
    bot = TsukimiBurger('Otsukimi')
    bot.strategy()
