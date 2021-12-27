#!/usr/bin/python


import rospy
import rosbag
from os.path import join
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

DEFAULT_IK_SERVICE = "/compute_ik"
DEFAULT_JOINT_STATES = "/joint_states"


class RecordFromEndEffector():

    def __init__(self, pose_topics=[], groups=[]):
        """Initialize class.
        @arg pose_topics list of strings containing the topics where
            to get from PoseStamped messages of the end effector pose.
        @arg groups list of strings containing the name of the MoveIt!
            groups to use to get the IK position of the poses given.
        Every pose_topics element will be mapped to a group, i.e.,
        the first topic in pose_topics will be the position to achieve
        by the first group in groups."""
        rospy.loginfo("Init LearnFromEndEffector()")        
        
        # Initiliaze recording variables
        # Initializing start as False. When start is True, the CB function
        # will append the topic msgs to a list
        self.start = False
        self.pose_subs = []
        self.pose_accumulators = []
        self.pose_topics = pose_topics
        self.groups = groups
        self.motion_name = "no_motion_name"
        self.current_bag_name = "no_bag_name"
        self.rosbag_file_path = '/home/roy/catkin_ws/src/roy_dmp/data/rosbag_recordings'
        self.end_effector_subs = "/end_effector_pose"
        # Subscribe to a PoseStamped topics
        rospy.loginfo("Subscribing to topics...")
        for idx, pose_topic in enumerate(pose_topics):
           rospy.loginfo("Subscribing to '" + pose_topic + "'...")
           subs = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb, callback_args=idx)
           self.pose_subs.append(subs)
           rospy.loginfo("Successful subscription to [" + str(idx) + "] '" + pose_topic + "'.")
           self.pose_accumulators.append([])       


    def pose_cd(self, data, cb_args):
        """Callback functions for PoseStamped messages.
        cb_args contains the idx of the topic to know which callback is which"""
        rospy.loginfo("Received from [" + str(cb_args) + "] " + self.pose_topics[cb_args] + ":\n  " + str(data))
        if self.start:
            self.pose_accumulators[cb_args].append(data)
    def start_record(self, motion_name, bag_name):
        self.motion_name = motion_name
        self.current_bag_name = bag_name
        self.start = True
        

    def stop_record(self):
        self.start = False
        for idx, pose_topic in enumerate(self.pose_topics):
            rospy.loginfo("Unsubscribing to '" + pose_topic + "'...")
            self.pose_subs[idx].unregister()
        # Write rosbag
        rospy.loginfo("Recording to a rosbag")
        file = join(self.rosbag_file_path, self.current_rosbag_name)
        self.current_rosbag = rosbag.Bag(file + '.bag', 'w')
        # self.current_rosbag = rosbag.Bag(self.current_bag_name + '.bag','w')
        for idx, poselist in enumerate(self.pose_accumulators):
            for ps in poselist:
                self.current_bag.write(self.pose_topics[idx], ps, t=ps.header.stamp)
        self.current_bag.close()
        rospy.loginfo("The recording fo the motion is finished, and the bag is closed")
        motion_data = {'motion_name' : self.motion_name,
                       'groups' : self.groups, # Get joints from group?
                       'rosbag_name': self.current_bag_name + '.bag'}

        return motion_data

class RecordFromJointState():
    """Manage the learning from joint positions"""
    def __init__(self):
        """Initialize class.
        @arg joint_names list of strings with the name of the
        joints to subscribe on joint_states."""
        rospy.loginfo("Init LearnFromJointState()")
        self.joint_states_topic = DEFAULT_JOINT_STATES

        self.start_recording = False
        self.joint_states_subs = rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_cb)
        rospy.loginfo("Connected.")
        self.current_rosbag_name = "uninitialized_rosbag_name"
        self.last_joint_states_data = None
        self.joint_states_accumulator = []
        self.motion_name = "no_motion_name"
        self.joints_to_record = []
        self.rosbag_file_path = '/home/roy/catkin_ws/src/roy_dmp/data/rosbag_recordings'        

        # Callback function for the ros_msgs. If recording == True. It appends the data
    def joint_states_cb(self, data):
        """joint_states topic callback """
        rospy.logdebug("Received joint_states:\n " + str(data))
        if self.start_recording:
            self.joint_states_accumulator.append(data)

        # Calling this function will activate the recording. 
        # It also gives the name of the outputfile.
    def start_record(self, motion_name, joints=[], bag_name="no_bag_name_set"):
        """ Start the recording of the joint states, and accumulate the msgs """
        self.current_rosbag_name = bag_name
        self.motion_name = bag_name
        self.start_recording = True
        if len(joints) > 0:
            self.joints_to_record = joints
        else:
            rospy.logerr(" No joints are given for recording. ABORTING")
            return
        
        # Stops the recording, and saving the appended data to a rosbag file.
    def stop_record(self):

        self.start_recording = False
        self.joint_states_subs.unregister()
        rospy.loginfo("Recording in a rosbag")
        file = join(self.rosbag_file_path, self.current_rosbag_name)
        self.current_rosbag = rosbag.Bag(file + '.bag', 'w')
        for js_msg in self.joint_states_accumulator:
            self.current_rosbag.write(DEFAULT_JOINT_STATES, js_msg, t=js_msg.header.stamp)
        self.current_rosbag.close()
        rospy.loginfo("Motion finished and closed bag.")
        motion_data = {'motion_name' : self.motion_name,
                       'joints' : self.joints_to_record,
                       'rosbag_name': self.current_rosbag_name + '.bag'}
        return motion_data        
        
if __name__=='__main__':
    """ Testing if the recording of the desirec topics are performend
    and that it is correctly saved to a rosbag for lager use.
    The test needs the topics to subscribe to and a list of joint names """
    
    rospy.init_node("Testing_recording_classes")
    rospy.loginfo("Initializing test of recordings")

    TEST_TOPIC_1 = "/joint_states"
    TEST_GROUP_1 = "/arm_controller"
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    js_recording = RecordFromJointState()
    rosbag_name_input = raw_input("Type the name of the new rosbag file recording\n")
    raw_input("Type enter to start recording")
    js_recording.start_record('Testing',JOINT_NAMES,rosbag_name_input)

    raw_input("Type enter to stop recording")
    # rospy.sleep(5)
    motion_data =  js_recording.stop_record()
    print(motion_data)