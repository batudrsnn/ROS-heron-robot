from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus
from copy import deepcopy
import moveit_commander
import sys
import threading
import tf
import rospy
import controller_manager_msgs.srv
from cartesian_trajectory_generator.msg import TrajectoryAction, TrajectoryGoal
import cartesian_trajectory_generator.srv
from geometry_msgs.msg import Pose, PoseStamped



class Manipulator():
    def __init__(self, robot_description="heron_robot", planning_group="manipulator", end_effector_link="ur5e_ee_link") -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot_commander = moveit_commander.RobotCommander()
     
        self.scene = moveit_commander.PlanningSceneInterface() 

        self.group = moveit_commander.MoveGroupCommander(planning_group) #arm
        self.group.set_end_effector_link(end_effector_link)
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3) 


        self.base_frame = "ur5e_base_link"

        self.tf = tf.TransformListener()

    def goto_pose_base(self, pose, planning_time=10):
        # pose here is in base frame coordinates
        self.group.set_pose_target(pose)
        print(pose)
        self.group.set_planning_time(planning_time)
        success = self.group.go(wait=True)
        return success       

    def goto_pose(self, pose, planning_time=10):
        # pose here is in arbitrary frame and will be transformed to base_frame 
        self.group.set_planning_time(planning_time)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = pose
        pose_base_frame = self.tf.transformPose(self.base_frame, pose_stamped)
        return self.goto_pose_base(pose_base_frame, planning_time)

    def goto_joint_state(self, joint_states, planning_time=10):
        # setting joint target in group and executing
        #print("============ Printing robot state")
        #print(self.robot_commander.get_current_state())
        #print("")
        self.group.set_joint_value_target(joint_states)
        self.group.set_planning_time(planning_time)
        return self.group.go(wait=True)

#class Manipulator:
    #def __init__(self):
       # """
       # TASK:
       #     Set up the variables etc. that you need in order to perform manipulation tasks.
       #     This is also a good place to manage moveit settings.
      #  """
      #  pass

    #def goto_joint_state(self, joint_state, planning_time=1):
    #    """
     #   TASK:
      #      Implement this.
      #      Given a list of joint states, this function should have moveit
      #      move the arm into those joint states.

       # HINTS:
       #     You can pass the joint_state variable directly into moveit.

      #  Parameters:
      #      pose: a list of joint states [s1, s2, s3, s4, s5, s6]
  #          planning_time: the time (in sections) that moveit is allowed to plan for.

   #     Returns:
    #        A boolean representing success/fail.
     #   """
        #return False


#################################################################################
# Arm to home position
#################################################################################

class ArmHome(SkillDescription):

    def createDescription(self):
        pass

class arm_home(PrimitiveBase):
    
    def createDescription(self):
        self.setDescription(ArmHome(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):
        joint_state_home = [
            1.5704865455627441,
            0.0031444269367675304,
            -2.8347482681274414,
            -0.32037051141772466,
            1.570810317993164,
            1.570784568786621
        ]

        manipulator = Manipulator()
        self.result = manipulator.goto_joint_state(joint_state_home)
        self.done = True

    def onStart(self):
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True
        

    def execute(self):
        if not self.done:
            return self.step('Running...')

        self.thread.join()

        if self.result:
            return self.success('Done')
        else:
            return self.fail('Failed', -1)

#################################################################################
# Arm to lookout position
#################################################################################

class ArmLookout(SkillDescription):

    def createDescription(self):
        pass

class arm_lookout(PrimitiveBase):
    
    def createDescription(self):
        self.setDescription(ArmLookout(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):
        joint_state_lookout = [
            3.1415,
            -1.5708,
            -1.5708,
            -1.5708,
            1.5708,
            2.3562
        ]

        manipulator = Manipulator()
        self.result = manipulator.goto_joint_state(joint_state_lookout)
        self.done = True

    def onStart(self):
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True
        

    def execute(self):
        if not self.done:
            return self.step('Running...')

        self.thread.join()

        if self.result:
            return self.success('Done')
        else:
            return self.fail('Failed', -1)

#################################################################################
# Go To Linear with Action
#################################################################################


class LinearMovement(SkillDescription):
    """
    @brief      Any arm movement that brings the end-effector at the
                target pose
    """
    def createDescription(self):
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)


class go_to_linear(PrimitiveActionClient):
    """
    @brief Move arm directly to target (aligns x y z)
    """

    def createDescription(self):
        self.setDescription(LinearMovement(), self.__class__.__name__)

    def buildClient(self):
        return actionlib.SimpleActionClient("/cartesian_trajectory_generator/goal_action",
                                            TrajectoryAction)

    def transformToFrame(self, element, target_frame):
        reasoner = self.wmi.get_reasoner('AauSpatialReasoner')
        frame_id = element.getProperty('skiros:FrameId').value
        reasoner.get_transform(frame_id, target_frame)
        reasoner.transform(element, target_frame)
        return element

    def buildGoal(self):
        """
        TASK:
            Implement transformation of the target pose into the map frame and add it
            the action goal.
            In the coordinate frame of the gripper, the z-axis points away from the gripper.
            E.g. when picking from a table, the z-axis would point into the table.

        HINTS:
            ROS has tf as a library for that.

        Parameters:
            goal: Action goal.
            target_pose: geometry_msgs::PoseStamped with the goal
        """
        target = self.params["Target"].value
        target_pose = target.getData(":PoseStampedMsg")
        goal = TrajectoryGoal()
        target = self.transformToFrame(target, 'map')
        target_msg = target.getData(':PoseStampedMsg')
        goal.goal = target_msg
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'

        return goal

    def onFeedback(self, msg):
        return self.step("Progress: {}%".format(round(100 * msg.time_percentage)))

    def onDone(self, status, msg):
        if status == GoalStatus.ABORTED:
            return self.fail("Failed aborted", -2)
        elif status == GoalStatus.SUCCEEDED:
            return self.success("Succeeded")
        elif status == GoalStatus.REJECTED:
            rospy.loginfo(status)
            rospy.loginfo(msg)
            return self.fail("Goal was rejected by action server.", -2)
        else:
            rospy.loginfo(status)
            rospy.loginfo(msg)
            return self.fail("Unknown return code.", -100)


#################################################################################
# Switch Controller
#################################################################################

class SwitchController(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Controller", "", ParamTypes.Required, [], "compliant|joint_config")


class switch_controller(PrimitiveBase):

    def createDescription(self):
        self.setDescription(SwitchController(), self.__class__.__name__)

    def onInit(self):
        self.c_manager_l = rospy.ServiceProxy('/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
        self.c_manager_s = rospy.ServiceProxy('/controller_manager/switch_controller', controller_manager_msgs.srv.SwitchController)

    def onStart(self):
        self.controllers = {}
        self.controllers["joint_config"] = "scaled_pos_traj_controller"
        self.controllers["compliant"] = "cartesian_compliance_controller"
        return True

    def execute(self):
        if self.params["Controller"].value not in self.controllers:
            return self.fail("Wrong controller type. Types are: compliant|joint_config", -1)
      
        # Check which one is running
        c_list = self.c_manager_l(controller_manager_msgs.srv.ListControllersRequest())
        c_states = {}
        for c in c_list.controller:
            if c.name in self.controllers.values():
                c_states[c.name] = c.state
        if self.controllers[self.params["Controller"].value] not in c_states:
            return self.fail("Desired controller '{}' not known to controller manager".format(self.controllers[self.params["Controller"].value]), -2)
        if c_states[self.controllers[self.params["Controller"].value]] == "running":
            return self.success("'{}' controller '{}' is already running.".format(self.params["Controller"].value, self.controllers[self.params["Controller"].value]))

        # Switch controllers
        req = controller_manager_msgs.srv.SwitchControllerRequest()
        req.start_controllers.append(self.controllers[self.params["Controller"].value])
        running_c = list(c_states.keys())[list(c_states.values()).index("running")]
        req.stop_controllers.append(running_c)
        req.strictness = 2  # Strict
        req.timeout = 1.    # Seconds 

        if self.c_manager_s(req):
            return self.success("Controller changed to '{}' with '{}'".format(self.params["Controller"].value, self.controllers[self.params["Controller"].value]))
        else:
            return self.fail("Failed switching controllers.", -3)
