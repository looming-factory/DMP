#!/usr/bin/env python

import rospy
import subprocess, yaml
from os.path import join
import numpy as np
import rosbag
from tf.transformations import euler_from_quaternion
from dmp.srv import GetDMPPlan, GetDMPPlanRequest,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
from dmp.msg import DMPTraj, DMPData, DMPPoint
from sensor_msgs.msg import JointState
import time
DEFAULT_JOINT_STATES = "/joint_states"
DEFAULT_FK_SERVICE = "/compute_fk"
DEFAULT_IK_SERVICE = "/compute_ik"

class motionGeneration():

    def __init__(self):
        self.arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.motion_difference = []
        self.weights_file_path = '/home/roy/catkin_ws/src/roy_dmp/data/weights'
        self.rosbag_file_path = '/home/roy/catkin_ws/src/roy_dmp/data/rosbag_recordings'

    def getNamesAndMsgList(self, joints, joint_state_msg):
        """ Get the joints for the specified group and return this name list and a list of it's values in joint_states
        Note: the names and values are correlated in position """
        list_to_iterate = joints
        curr_j_s = joint_state_msg
        ids_list = []
        msg_list = []
        rospy.logdebug("Current message: " + str(curr_j_s))
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            msg_list.append(curr_j_s.position[idx_in_message])
        rospy.logdebug("Current position of joints in message: " + str(ids_list))
        rospy.logdebug("Current msg:" + str(msg_list))

        return list_to_iterate, msg_list        


   

    def loadMotionFromJointStates(self, bagname, joints,_dt=000.8,_K=100,_D=2.0 * np.sqrt(100),_num_bases=None):
        """Load motion from the bag name given """
        # Get bag info
        file = join(self.rosbag_file_path,bagname)
        self.info_bag = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', file],stdout=subprocess.PIPE).communicate()[0])

        # Create a DMP from the number of joints in the trajectory
        dims = len(joints)
        dt = _dt
        K = _K
        D = _D
        num_bases = _num_bases

        traj = []
        bag = rosbag.Bag(file)
        first_point = True
        for topic, msg, t in bag.read_messages(topics=[DEFAULT_JOINT_STATES]):

            # Process interesting joints
            names, positions = self.getNamesAndMsgList(joints,msg)
            # Append interesting joints here
            traj.append(positions)
            if first_point:
                # Store the first point
                self.motion_x0 = positions
                first_point = False
        bag.close()
        # Store the final point
        self.motion_goal = positions

        # Compute the difference between initial and final point
        for val1, val2 in zip(self.motion_x0, self.motion_goal):
            self.motion_difference.append(val2-val1)
        print(str(len(traj)) + " points in example traj. Using " + str(num_bases) + " num_bases")

        resp = self.makeLFDRequest(dims,traj,dt,K,D,num_bases)
        # Set it as the active DMP on the DMP server( ros_DMP)
        self.makeSetActiveRequest(resp.dmp_list)
        self.resp_from_makeLFDRequest = resp

        rospy.loginfo("Joints:" + str(joints))
        rospy.loginfo("Initial pose:" + str(self.motion_x0))
        rospy.loginfo("Final pose: " + str(self.motion_goal))
        time = self.info_bag['duration']
        rospy.loginfo("Time: " + str(time))
        #rospy.loginfo("DMP result: " + str(self.resp_from_makeLFDRequest))
        motion_dict = self.saveMotionYAML(bagname + ".yaml", bagname, joints, self.motion_x0, self.motion_goal, self.resp_from_makeLFDRequest, time)
        return motion_dict        


    def saveMotionYAML(self, yamlname, name, joints, initial_pose, final_pose, computed_dmp, time):
        """Given the info of the motion computed with the DMP server save it into a yaml file.
        @yamlname string name of the yaml file
        @name string name of the motion
        @joints list of strings joints that are included in this motion
        @initial_pose list of double initial pose for the motion as it was recorded in the training
        @final_pose list of double final pose of the motion as it was recorded in the training
        @computed_dmp python/object/new:dmp.srv._LearnDMPFromDemo.LearnDMPFromDemoResponse with the response of the DMP server
        @time double how long the motion took"""
        motion_dict = {"name": name,
                        "joints" : joints,
                        "initial_pose" : initial_pose,
                        "final_pose" : final_pose,
                        "computed_dmp" : computed_dmp,
                        "duration" : time}
        rospy.loginfo("motion_dict:\n" + str(motion_dict))
        file = join(self.weights_file_path, yamlname)
        try:
            with open(file, "w") as f:
                yaml.dump(motion_dict,f)
            self.result = "success"
        except:
            rospy.logerr("Cannot save weight file, Check if the directory of the weight file exist")
            self.result = "failed"
        return motion_dict        

    def loadMotionYAML(self, yamlname):
        """Given a yamlname which has a motion saved load it and set it as active in the DMP server"""
        file = join(self.weights_file_path, yamlname)

        try:
            with open(file, "r") as f:
                motion_dict = yaml.load(f)
        except:
            print("Can not find file: " + yamlname)
            return None 
        # set it as the active DMP on the dmp server
        self.makeSetActiveRequest(motion_dict['computed_dmp'].dmp_list)
        self.resp_from_makeLFDRequest = motion_dict['computed_dmp']
        return motion_dict

    def makeLFDRequest(self, dims, traj, dt, K_gain, D_gain, num_bases):
        """Learn a DMP from demonstration data """
        demotraj = DMPTraj()

        for i in range(len(traj)):
            pt = DMPPoint()
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims

        print("Starting ...")
        init_time = time.time()
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj,k_gains,d_gains,num_bases)
        except rospy.ServiceException as e:
            print("Service call failes: %s"%e)
        fin_time = time.time()
        print("LfD done, took: " + str(fin_time - init_time))

        return resp

    def makeSetActiveRequest(self, dmp_list):
        """ Set a DMP as active on the DMP server"""
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

    def getPlan(self, initial_pose, goal_pose, seg_length=-1, initial_velocities=[], t_0 = None, tau=5, dt=0.008, integrate_iter=1,goal_thresh=[]):
        """Generate a plan..."""

        x_0 = initial_pose
        x_dot_0 = [0.0] * len(initial_pose)
        t_0 = 0
        this_dt= dt
        this_tau = tau*2
        this_integrate_iter = integrate_iter
        goal = goal_pose
        if len(goal_thresh) > 0:
            this_goal_thresh = goal_thresh
        else:
            this_goal_thresh = [0.01] * len(initial_pose)
        seg_length = seg_length          #Plan until convergence to goal is -1

        rospy.logwarn("tau is: " + str(this_tau))

        plan_resp = self.makePlanRequest(x_0, x_dot_0, t_0, goal, this_goal_thresh,
                               seg_length, this_tau, this_dt, this_integrate_iter)
        return plan_resp

    def makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh,
                        seg_length, tau, dt, integrate_iter):
        """Generate a plan from a DMP """
        print("Starting DMP planning...")
        init_time = time.time()
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
                       seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
        fin_time = time.time()
        print ("DMP planning done, took: " + str(fin_time - init_time))
    
        return resp

if __name__ == "__main__":

    import time
    import roslib; roslib.load_manifest('ur_driver')
    import rospy
    import math
    import actionlib
# from ros_dmp.msg import *
    from control_msgs.msg import *
    from trajectory_msgs.msg import *
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Pose
    from nav_msgs.msg import Path
    from math import pi
    rospy.init_node("test_generation_classes")
    rospy.loginfo("Initializing dmp_generation test.")
    # client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print("Waiting for server...")
    client.wait_for_server()

    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    mg = motionGeneration()
    mg.loadMotionFromJointStates("recording_start1.bag",JOINT_NAMES)
    joint_states = rospy.wait_for_message("joint_states",JointState)
    initial_pose = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
    # initial_pose =[-0.2273033300982874, -2.298889462147848, -1.0177272001849573, -1.3976243177997034,  1.5502419471740723, 9.261386219655172]
    final_pose = [-2.3324595133410853, -2.2434170881854456, -1.1172669569598597, -1.3543337027179163, 1.5941375494003296, 7.169057373200552]
    pla = DMPTraj()
    pla = mg.getPlan(initial_pose,final_pose,-1,[],None,tau=5,dt=0.008)
    # print(joint_states.position[0])
    # print(type(joint_states))


    # g = FollowJointTrajectoryGoal()
    # g.trajectory = JointTrajectory()
    # g.trajectory.joint_names = JOINT_NAMES

    # try:
    #     # joint_states = rospy.wait_for_message("joint_states",JointState)
    #     # joints_pos = [-0.2273033300982874, -2.298889462147848, -1.0177272001849573, -1.3976243177997034, 1.5502419471740723, 9.261386219655172]
    #     # joint_value = pla.plan.points.positions
    #     # velocity = pla.points.velocities
    #     times = pla.plan.times
    #     d= 2.00
    #     g.trajectory.points = [JointTrajectoryPoint(positions=initial_pose, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
    #     for i in range(len(pla.plan.times)):
    #         joints_pos = pla.plan.points[0].positions
    #         joint_value = pla.plan.points[i].positions
    #         velocity = pla.plan.points[i].velocities
    #         # print(i)
    #         Q = [joint_value[0],joint_value[1],joint_value[2],joint_value[3],joint_value[4],joint_value[5]]
    #         V = [velocity[0],velocity[1],velocity[2],velocity[3],velocity[4],velocity[5]]
    #         T = times[i]
    #         # print(Q)
    #         # print(T)
    #         # print(pla.plan.points[0].positions)
    #         # print(initial_pose)
    #         g.trajectory.points.append(JointTrajectoryPoint(positions=Q, velocities=V, time_from_start=rospy.Duration(T)))
    #     client.send_goal(g)
    #     client.wait_for_result(rospy.Duration(0))
       
    # except KeyboardInterrupt:
    #     client.cancel_goal()
    #     raise
    # except:
    #     raise