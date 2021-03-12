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
from sensor_msgs.msg import LaserScan

from obstacle_detector.msg import Obstacles

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

MARKER_TARGETS = {"Tomato_N":TPos(0.9,0.4,math.pi*11/12),"Tomato_S":TPos(0.0,0.5,0.0), "Omelette_N":TPos(0.9,-0.4,math.pi*13/12),"Omelette_S":TPos(0.0,-0.5,0.0),
    "Pudding_N":TPos(0.0,0.5,math.pi),"Pudding_S":TPos(-0.9,0.4,math.pi/12), "OctopusWiener_N":TPos(0.0,-0.5,math.pi),"OctopusWiener_S":TPos(-0.9,-0.4,-math.pi/12),
    "FriedShrimp_N":TPos(0.6,0.0,math.pi),"FriedShrimp_E":TPos(0.0,-0.5,math.pi/2),"FriedShrimp_W":TPos(0.0,0.5,-math.pi/2),"FriedShrimp_S":TPos(-0.6,0.0,0.0)}



class EnemyDetector:
    def __init__(self):
        self.max_distance = 1.0
        self.thresh_corner = 0.25
        self.thresh_center = 0.35
        self.thresh_vel = 0.08

        self.enemy_pose_x = 0
        self.enemy_pose_y = 0
        self.enemy_th = 0

        self.is_enemy_detected = False

        # subscriber
        self.obstacle_sub = rospy.Subscriber('obstacles', Obstacles, self.obstacleCallback)


    def obstacleCallback(self,data):
        for obs in data.circles:
            x = obs.center.x
            y = obs.center.y
            vel_x = obs.velocity.x
            vel_y = obs.velocity.y
            vel = (vel_x**2 + vel_y**2)**0.5
            # radius = obs.radius
            if self.is_point_enemy(x,y,vel):
                # print("enemey velocity: " + str(vel))
                self.enemy_pose_x = x
                self.enemy_pose_y = y
                self.enemy_th = math.atan2(vel_y,vel_x)
                self.is_enemy_detected = True
                # print("enemy pose (x,y): " + str(self.enemy_pose_x) + "," + str(self.enemy_pose_y))
                return
            else:
                # print("obstacle velocity: " + str(vel))
                pass
        self.is_enemy_detected = False


    def findEnemy(self, pose_x, pose_y):
        '''
        input robot locate(pose_x, pose_y)
        return is_near_enemy(BOOL), enemy_direction[rad](float), enemy_distance(float)
        '''
        if self.is_enemy_detected:
            enemy_distance = math.sqrt((pose_x - self.enemy_pose_x)**2 + (pose_y - self.enemy_pose_y)**2)
            enemy_direction = math.atan2((self.enemy_pose_y - pose_y),(self.enemy_pose_x - pose_x))
            is_near_enemy = enemy_distance < self.max_distance
        else:
            is_near_enemy = False
            enemy_distance = None
            enemy_direction = None

        print("Enemy: {}, Distance: {}, Direction: {}".format(is_near_enemy, enemy_distance, enemy_direction))
        return is_near_enemy, enemy_distance, enemy_direction

    # respect is_point_enemy from team rabbit
    # https://github.com/TeamRabbit/burger_war
    def is_point_enemy(self, point_x, point_y, vel):
        #物体の速度をチェック
        if vel > self.thresh_vel:
            return True
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



class TsukimiBurger():
    def __init__(self, bot_name):
        # bot name
        self.name = bot_name
        self.side_color = rosparam.get_param("/searchRun/side")
        # robot state
        """
        PENDING=0 ACTIVE=1 PREEMPTED=2 SUCCEEDED=3 ABORTED=4
        REJECTED=5 PREEMPTING=6 RECALLING=7 RECALLED=8 LOST=9
        """
        self.move_base_state = 0

        self.fUpdateRoute = False
        # robot pose
        self.pose_x = 0
        self.pose_y = 0
        self.th = 0

        # speed [m/s]
        self.speed = 0.08

        self.target_markers = []
        self.current_target_marker = None

        self.detected_player_markers = []

        self.scan = []

        # EnemyDetector
        self.enemy_detector = EnemyDetector()
        self.enemy_detect_timer = 0.0

        # actionlib
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # subscriber
        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        self.war_state_sub = rospy.Subscriber('war_state', String, self.warstateCallback)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

    def poseCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        self.th = rpy[2]

    # warstateから取得出来ていないマーカーを探してtarget_markersに格納
    def warstateCallback(self,data):
        state = json.loads(data.data)
        tmp = []
        pl = []
        print("color: " + self.side_color)
        for j in state["targets"]:
            if j["name"] in PLAYER_MARKERS:
                if j["player"] != "n":
                    pl.append(j["name"])
            elif j["player"] != self.side_color:
                tmp.append(j["name"])
        self.target_markers = self.sortTargetMarkers(tmp)
        self.detected_player_markers = pl

    def lidarCallback(self, data):
        scan = data.ranges
        self.scan = scan

    # target_markersに格納されているマーカーの内、一番近いものを計算
    def calcNearTarget(self,markers):
        index = 0
        min_dist = 100.0
        for t in range(len(markers)):
            tpos = MARKER_TARGETS[markers[t]]
            dist = ( (self.pose_x - tpos.x)**2 + (self.pose_y - tpos.y)**2 ) **0.5
            if dist < min_dist:
                index = t
                min_dist = dist
            # print(self.pose_x,self.pose_y, tpos.x , tpos.y)
        # print(self.target_markers[index])
        return markers[index]

    # target_markersに格納されているマーカーを近い順にソート
    def sortTargetMarkers(self,markers):
        res = []
        while len(markers) != 0:
            t = self.calcNearTarget(markers)
            res.append(t)
            markers.remove(t)
        return res

    def calcAvoidEnemyTwist(self,enemy_distance,enemy_direction, timer):
        twist = Twist()
        twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0

        print("distance: " + str(0.4 + timer/100.0*0.6))
        if enemy_distance > 0.4 + timer/100.0*0.6 :
            twist.linear.x = self.speed
            if self.isFrontNearWall(self.scan):
                twist.linear.x = twist.linear.x * -0.2
        else:
            twist.linear.x = -self.speed
            if self.isRearNearWall(self.scan):
                twist.linear.x = twist.linear.x * -0.2

        if self.isContactWall(self.scan):
            twist.linear.x = twist.linear.x * 0.5

        th_diff = enemy_direction - self.th
        while not math.pi >= th_diff >= -math.pi:
            if th_diff > 0:
                th_diff -= 2*math.pi
            elif th_diff < 0:
                th_diff += 2*math.pi
        # print("th: {}, enemy_direction: {}, th_diff: {}".format(self.th, enemy_direction, th_diff))

        if twist.linear.x > 0:
            th_delta = self.calcDeltaTheta(th_diff)
        else:
            th_delta = self.calcDeltaTheta(th_diff + math.pi)

        th_diff = th_diff + th_delta
        twist.angular.z = max(-0.5, min(th_diff , 0.5))

        return twist

    # respect calcDeltaTheta from sample program level_3_clubhouse
    # 進行方向の障害物を避けるように角度を調整
    def calcDeltaTheta(self, th):
        if not self.scan:
            return 0.
        R0_idx = self.radToidx(th - math.pi/8)
        R1_idx = self.radToidx(th - math.pi/4)
        L0_idx = self.radToidx(th + math.pi/8)
        L1_idx = self.radToidx(th + math.pi/4)
        R0_range = 99. if self.scan[R0_idx] < 0.1 else self.scan[R0_idx]
        R1_range = 99. if self.scan[R1_idx] < 0.1 else self.scan[R1_idx]
        L0_range = 99. if self.scan[L0_idx] < 0.1 else self.scan[L0_idx]
        L1_range = 99. if self.scan[L1_idx] < 0.1 else self.scan[L1_idx]

        if R0_range < 0.3 and L0_range > 0.3:
            return math.pi/4
        elif R0_range > 0.3 and L0_range < 0.3:
            return -math.pi/4
        elif R1_range < 0.2 and L1_range > 0.2:
            return math.pi/8
        elif R1_range > 0.2 and L1_range < 0.2:
            return -math.pi/8
        else:
            return 0.

    def isFrontNearWall(self, scan):
        if not len(scan) == 360:
            return False
        forword_scan = scan[:15] + scan[-15:]
        # drop too small value ex) 0.0
        forword_scan = [x for x in forword_scan if x > 0.1]
        if len(forword_scan) == 0:
            return False
        if min(forword_scan) < 0.2:
            return True
        return False
    def isRearNearWall(self, scan):
        if not len(scan) == 360:
            return False
        rear_scan = scan[180-15:180+15]
        # drop too small value ex) 0.0
        rear_scan = [x for x in rear_scan if x > 0.1]
        if len(rear_scan) == 0:
            return False
        if min(rear_scan) < 0.2:
            return True
        return False
    def isContactWall(self, scan):
        if not len(scan) == 360:
            return False
        forword_scan = scan[:15] + scan[-15:]
        rear_scan = scan[180-15:180+15]
        num = sum(x<0.3 for x in forword_scan) + sum(x<0.3 for x in rear_scan)
        print("wall points:" + str(num))
        if num > 20:
            print("Stop until the enemy moves")
            return True
        return False


    def radToidx(self, rad):
        deg = int(rad / (2*math.pi) * 360)
        while not 360 > deg >= 0:
            if deg > 0:
                deg -= 360
            elif deg < 0:
                deg += 360
        return deg



    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()
        # sprint(self.client.get_state())

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

        # wait = self.client.wait_for_result()
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     return self.client.get_result()

    def setNearGoal(self):
        if len(self.target_markers) > 0:
            marker = self.target_markers[0]
            self.current_target_marker = marker
            self.setGoal(MARKER_TARGETS[marker].x,MARKER_TARGETS[marker].y,MARKER_TARGETS[marker].rot)

    def isRecognizedMarker(self):
        if len(self.target_markers) == 0:
            return False
        res = not self.current_target_marker in self.target_markers
        # print("current: {}, markers: {}, res: {}".format(self.current_target_marker,self.target_markers,res))
        if res:
            self.current_target_marker = None
        return res

    def isDetectedAllMyMarkers(self):
        markers = []
        if self.side_color == "r":
            tmp = ["RE_B","RE_L","RE_R"]
            markers = [x for x in self.detected_player_markers if x in tmp]
            # print("maekers :" +str(markers))
        elif self.side_color == "b":
            tmp = ["BL_B","BL_L","BL_R"]
            markers = [x for x in self.detected_player_markers if x in tmp]

        if len(markers) == 3:
            return True
        else:
            return False

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        Go and Back loop forever
        '''
        r = rospy.Rate(5) # change speed 1fps

        while not rospy.is_shutdown():
            is_near_enemy,enemy_dist,enemy_direct = self.enemy_detector.findEnemy(self.pose_x, self.pose_y)
            self.move_base_state = self.client.get_state()
            print("move_state: " + str(self.move_base_state))
            print("detected_player_markers: " + str(self.detected_player_markers))
            # 敵ロボットが近辺にいる場合
            if is_near_enemy and not self.isDetectedAllMyMarkers():
                if self.move_base_state != 2:
                    self.client.cancel_goal()
                twist = self.calcAvoidEnemyTwist(enemy_dist,enemy_direct, self.enemy_detect_timer)
                self.vel_pub.publish(twist)

                self.enemy_detect_timer = min(self.enemy_detect_timer + 1.0, 100.0)


            # 通常のフィールド周回
            else:
                if self.isRecognizedMarker():
                    self.client.cancel_goal()

                if self.move_base_state == 0 or self.move_base_state == 2 or self.move_base_state == 4 or self.move_base_state == 9:
                    self.setNearGoal()
                elif self.move_base_state == 3:
                    if self.current_target_marker:
                        self.target_markers.pop(0)
                        self.current_target_marker = None
                    self.setNearGoal()

                self.enemy_detect_timer = max(self.enemy_detect_timer - 2.0, 0.0)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('Otsukimi_run')
    bot = TsukimiBurger('Otsukimi')
    bot.strategy()
