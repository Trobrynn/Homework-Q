#!/usr/bin/env python
# -*- coding: UTF-8 -*-

""" 
简单的实现多点导航的功能
      
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from std_msgs.msg import String

locations = dict()
locations['bookshelf'] = Pose(Point(3.5, 4, 0.000), Quaternion(0.000, 0.000, -0.975891, 0.218259))
locations['door'] = Pose(Point(-5, 1.2, 0.000), Quaternion(0.000, 0.000, 0.625259, 0.780417))
locations['can'] =Pose(Point(-0.5,-2.5, 0.000), Quaternion(0.000, 0.000, -0.934679, 0.355494))


# 判断字符串是否相等
def str_eual(a,b):
    a_ = "".join(a.split())
    b_ = "".join(b.split())
    flag = a_ == b_
    return flag
class NavTest():
    def __init__(self):
        # -----------------初始化节点-----------------
        rospy.init_node('nav_core', anonymous=True)
        # ----------------建立响应函数-----------------
        rospy.on_shutdown(self.shutdown)
        # 控制频率
        self.rate = rospy.Rate(1)
        self.pubword=False 
        # 速度控制话题,用于速度缓冲
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=5)
        # 目标点状态
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        # --------------------定义初始的目标序列--------------------
        self.target_goals = dict()
        # TODO:需要初始定义的地方
        self.init_position()
        # 记录当前任务下需要前往的目标点
        self.current_goal = Pose()
        # 是否开始导航
        self.start_navigaiton = False 
        # 是否得到控制命令
        self.get_control_order = False
        #---------------------用于记录任意时刻的位置与姿态-------------
        # 按照需求记录位姿
        self.get_current_pose = False
        # 如果需要的时候就把当前的位姿记录下来
        rospy.Subscriber("present_pose",Pose,self.pose_sub,queue_size=1)
        #rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.pose_sub,queue_size=1)

        # 同时记录目标位置标记
        self.current_pose_label = str()
        #--------------------------------链接move_base 服务------------------------------
        # 链接服务器
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        #-------------------------------设置初始位姿--------------------------------------
        #-------------------------------阻塞进程-----------------------------------------
        # 阻塞进程,获取初始位姿
        self.initial_pose = PoseWithCovarianceStamped()        
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        self.pub_current_pose = rospy.Publisher('initialpose',PoseWithCovarianceStamped,queue_size=1)
        
        # 阻塞进程
	while self.initial_pose.header.stamp == "":
            rospy.sleep(1)
        #-------------------------设置初始变量------------------------------------------------
        # 下面都是一些简单的信息,用于记录导航状态
        i = 0
        n_goals = 0
        n_successes = 0
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        # -------------------------等待控制流唤醒---------------------------------------------
        rospy.Subscriber("/goal",String,self.goal_callback,queue_size=1)
        self.control_pub = rospy.Publisher("/xfwakeup",String,queue_size=1)

        #---------------------等待goal唤醒---------------------------------
        self.last_location = Pose()        
        self.location = Pose()
        #----------------------开始导航----------------------------------------
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            if not self.start_navigaiton:
		if self.pubword==False:
		    self.pub_control()
		    self.pubword=True
                rospy.loginfo("Wait for new goal!")
                self.rate.sleep()                            
                continue
            # 避免重复进入
            self.start_navigaiton = False
	    rospy.loginfo("%s",self.current_goal)
            self.location = self.current_goal
            # 更新初始化位姿,实时更新位姿
	    if self.initial_pose.header.stamp == "":
                distance = sqrt(pow(self.location.position.x - 
                                    self.last_location.position.x, 2) +
                                pow(self.location.position.y - 
                                    self.last_location.position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
            distance = sqrt(pow(self.location.position.x - 
                            self.last_location.position.x, 2) +
                        pow(self.location.position.y - 
                            self.last_location.position.y, 2))
            self.initial_pose.header.stamp = ""
            # 存储上一个地点
            self.last_location = self.location
            # 增加计时器
            i += 1
            n_goals += 1
            # 跟新目标
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = self.location
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            # 终端输出
            rospy.loginfo("Going to: %f %f %f"%(self.location.position.x,self.location.position.y,self.location.position.z))
            # 开始机器人向下一个节点            
            self.move_base.send_goal(self.goal,done_cb= self.donecb,active_cb=self.activecb,feedback_cb=self.feedbackcb)
            # 容限时间
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            # 检查是否成果
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                    rospy.loginfo("Timed out achieving goal")
		    self.pub_control()
                else:
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            # 接收下一个命令
            self.get_control_order  =False
            self.rate.sleep()
    def goal_callback(self,msg):
        # 避免重复获取命令
        # 执行完成之后就可以转为False

	rospy.loginfo("goal_callback:%s",msg)   

            # 然后进行导航
        self.start_navigaiton = True
        self.current_goal = self.target_goals[msg.data]


    # TODO:设置当前的位姿为一预设姿态
    # BUG:预设一般姿态,需要测试
    def set_initial_pose(self,target_pose):
        self.init_position = PoseWithCovarianceStamped
        self.initial_pose.pose = target_pose
        self.initial_pose.header.stamp = rospy.Time.now()
        self.pub_current_pose.publish(self.initial_pose)
        return 
    # 初始化位置
    def init_position(self):
        # 初始识别客人的地方
        self.target_goals=locations       
        return 
    #def set_goal(name,target_pose):
    #    self.target_goals[str(name)] = target_pose
    #    return
    # 获取当前的位置和姿态
    # BUG:这个可能没有这么简单
    def pose_sub(self,msg):
        if self.get_current_pose:
	
	    self.target_goals[self.current_pose_label]=msg.pose.pose
            #self.set_goal(self.current_pose_label,msg.pose)
            self.get_current_pose = False
            rospy.loginfo("get %s : %f %f %f"%(self.current_pose_label,msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z))
            return 
        else:
            return 
    def donecb(self,state,result):
        return 
    def activecb(self):
        return 
    def feedbackcb(self,fb):
        if self.start_navigaiton :
            self.move_base.cancel_goal()
            rospy.loginfo("cancel_goal and send new goal")
        else:
            return
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    def pub_control(self,total_time = 5):
	print("pub_control")
        rospy.sleep(1)        
	strings="ok"
        for _ in range(total_time):
            self.control_pub.publish(strings)      
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
