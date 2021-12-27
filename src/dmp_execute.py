#!/usr/bin/python


from numpy.core.numeric import _full_like_dispatcher
import rospy
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayRobotState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg._DisplayRobotState import DisplayRobotState
import time
from moveit_msgs.srv import  GetPositionIK
from tf.transformations import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import sys
sys.path.append("/home/roy/catkin_ws/src/roy_dmp/include/roy_dmp")
from kinematics_interface import *
import roslib; roslib.load_manifest('ur_driver')
import actionlib
from trajectory_msgs.msg import *
from math import pi

DEFAULT_JOINT_STATES = '/joint_states'
EXECUTE_KNOWN_TRAJ_SRV = '/execute_kinematic_path'
DEFAULT_IK_SERVICE = "/compute_ik"


DEBUG_MODE =  True



class motionExecution():

    def __init__(self):
        rospy.loginfo("Initializing motionExecution")
        rospy.loginfo("Connecting to MoveIt! known trajectory executor server '" + EXECUTE_KNOWN_TRAJ_SRV + "'...")
        rospy.loginfo("Connected.")
        self.sv = StateValidity()
        self.fk = ForwardKinematics()
        self.ik = InverseKinematics()
        self.robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState,queue_size=1)
        rospy.sleep(0.1) # Give time to the publisher to register
        #TODO: make ik_service_name a param to load from a yaml
        self.imitated_path_pub = rospy.Publisher("/imitated_path", Path, queue_size=1)
        self.ik_service_name = DEFAULT_IK_SERVICE
        # Get a ServiceProxy for the IK service
        rospy.loginfo("Waiting for service '" + self.ik_service_name + "'...")
        rospy.wait_for_service(self.ik_service_name)
        self.ik_serv = rospy.ServiceProxy(self.ik_service_name, GetPositionIK)
        rospy.loginfo("Successful connection  to '" + self.ik_service_name + "'.")        
        
        self.arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
     
    def robotTrajectoryFromPlan(self, plan, joint_names):
        """Given a dmp plan (GetDMPPlanResponse) create a RobotTrajectory to be able to visualize what it consists and also
        to be able to send it to execution"""
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = joint_names
        for point, time in zip(plan.plan.points, plan.plan.times):
            jtp = JointTrajectoryPoint()
            jtp.positions = point.positions
            jtp.velocities = point.velocities
            jtp.time_from_start = rospy.Duration(time)
            rt.joint_trajectory.points.append(jtp)
        return rt

    def checkTrajectoryValidity(self, robot_trajectory, groups=[]):
        """Given a robot trajectory, deduce it's groups and check it's validity on each point of the traj
        returns True if valid, False otherwise
        It's considered not valid if any point is not valid"""

        init_time = time.time()
        if len(groups) > 0:
            groups_to_check = groups
        else:
            groups_to_check = ['manipulator'] # Automagic group deduction... giving a group that includes everything 
        for traj_point in robot_trajectory.joint_trajectory.points:
            rs = RobotState()
            rs.joint_state.name = robot_trajectory.joint_trajectory.joint_names
            rs.joint_state.position = traj_point.positions
            for group in groups_to_check:
                result = self.sv.getStateValidity(rs,group)
                if not result.valid:
                    rospy.logerr("Trajectory is not valid at point (RobotState):" + str(rs) + "with result of StateValidity: " + str(result))
                    rospy.logerr("published in /robot_collision_state the conflicting state")
                    return False
            fin_time = time.time()
        rospy.logwarn("Trajectory validity of " + str(len(robot_trajectory.joint_trajectory.points)) + " points took " + str(fin_time - init_time))
        return True

   
            



  
    def sendTrajectoryAction(self,pla,_initial_pose,simulation):
        if simulation:
            client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        else:
            client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        client.wait_for_server()
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.arm
        initial_pose = _initial_pose
        
        try:
            times = pla.plan.times
            d= 2.00
            g.trajectory.points = [JointTrajectoryPoint(positions=initial_pose, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
            for i in range(len(pla.plan.times)):
                joint_value = pla.plan.points[i].positions
                velocity = pla.plan.points[i].velocities
                Q = [joint_value[0],joint_value[1],joint_value[2],joint_value[3],joint_value[4],joint_value[5]]
                V = [velocity[0],velocity[1],velocity[2],velocity[3],velocity[4],velocity[5]]
                T = times[i]
                g.trajectory.points.append(JointTrajectoryPoint(positions=Q, velocities=V, time_from_start=rospy.Duration(T)))
            client.send_goal(g)
            client.wait_for_result(rospy.Duration(0))
        except KeyboardInterrupt:
            client.cancel_goal()
            raise
        except:
            print("Fault")
            raise
        return True

    def fkPath(self,_position,_linkName):
        fk_link_names = _linkName
        joint_names = self.arm
        position = _position
        fk_result = self.fk.getFK(fk_link_names,joint_names,position)
        return fk_result.pose_stamped[0].pose

    def get_IK_from_Quart(self,_ps):
        ik_link_name = "rg2_eef_link"
        group_name = "manipulator"
        ps = PoseStamped()
        ps.header.frame_id ="base_link"
        ps.pose = _ps
        ik_result = self.ik.getIK(group_name,ik_link_name,ps)
        return ik_result


    def pathPublish(self,_path,_linkName):
        imitated_path = Path()
        imitated_path.header.frame_id = "/base_link"

        for itr in range(len(_path.plan.points)):
            joint_positions = _path.plan.points[itr].positions
            path = self.fkPath(joint_positions,_linkName)
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = path.position.x
            pose_stamped.pose.position.y = path.position.y
            pose_stamped.pose.position.z = path.position.z
            imitated_path.poses.append(pose_stamped)
        self.imitated_path_pub.publish(imitated_path)


if __name__ == "__main__":

    rospy.init_node("test_execution_classes")
    rospy.loginfo("Initializing dmp_execution test.")
    me = motionExecution()
    # mg = motionGeneration()
    # joint_states = rospy.wait_for_message("joint_states",JointState)
    # # initial_pose = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
    # initial_pose =[-0.2273033300982874, -2.298889462147848, -1.0177272001849573, -1.3976243177997034,  1.5502419471740723, 9.261386219655172]
    # final_pose = [-2.3324595133410853, -2.2434170881854456, -1.1172669569598597, -1.3543337027179163, 1.5941375494003296, 7.169057373200552]
    # pla = mg.getPlan(initial_pose,final_pose,-1,[],None,tau=5,dt=0.008)
    # print(initial_pose)
    # robot_traj = me.robotTrajectoryFromPlan(pla,me.arm)
    # validity = me.checkTrajectoryValidity(robot_traj)
    # print(validity)
    # if(validity == True):
    #     print("Valid trajectory")
    #     st = me.sendTrajectoryAction(pla,initial_pose)
    #     print("finished")
    # elif(validity == False):
    #     print("Not a valid trajectory")
    me.fkPath()
    
          
