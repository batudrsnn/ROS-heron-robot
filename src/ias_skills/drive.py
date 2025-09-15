from skiros2_skill.core.skill import SkillDescription, ParamOptions, SkillBase, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import skiros2_common.tools.logger as log
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String
import rospy
import threading

class Navigation:
    def __init__(self):
        """
        TASK:
            Set up all the variables etc. you need in order to perform navigation tasks.
        """
        pass
        rospy.loginfo("navigation instance initiated")
        # create the connection to the action server
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("action client initiated")
        # waits until the action server is up and running
        self.client.wait_for_server()
        rospy.loginfo("action server ready")

    """
    TASK (OPTIONAL):
        Add more methods if needed.
    """

    def drive_to(self, pose, max_tries=5, sleep_seconds_between_tries=5.0):
        """
        TASK:
            Implement this function.
            Given a ROS Pose message, this function should make the robot drive to that location.

        TASK (OPTIONAL):
            Make the robot retry if it fails.
            In this case you should make use of the max_tries variable.
            Between each attempt, you should wait a number of seconds specified by
            the sleep_seconds_between_tries variable.

        Parameters:
            pose: A ROS Pose message object.
            max_tries: An integer describing the maximum number of driving attempts.
            sleep_seconds_between_tries: A float describing how long to sleep between attempts (in seconds).
        Returns:
            A boolean representing success/fail.
        """
        rospy.loginfo("drive to called")
        goal = MoveBaseGoal () #Creates the variable goal of the goal message type
        rospy.loginfo("Goal message object created")
        #Set goal to the input pose to the function
        goal.target_pose.pose=pose
        rospy.loginfo("target pose set as pose")
        
        #Set the frame id for it to work
        goal.target_pose.header.frame_id = 'map'
        rospy.loginfo("frame id set as map")
      
        #Send and wait until the goal is reached
        self.client.send_goal(goal)
        rospy.loginfo("goal sent, waiting for result")
        self.client.wait_for_result()
        rospy.loginfo("result received")
        state_result = self.client.get_state()
        rospy.loginfo(state_result)

        #Provide feedback on screen and to caller
        if state_result == 3 :
            rospy.loginfo("pose reached")
            return True
        else:
            rospy.loginfo("there was some error")
            return False
        
class Drive(SkillDescription):   
    def createDescription(self):
        # =======Params=========
        self.addParam("TargetLocation", Element("scalable:Workstation"),ParamTypes.Required)   

class drive(PrimitiveBase):

    def createDescription(self):
        self.setDescription(Drive(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):
        navigation = Navigation()

        target_location = self.params["TargetLocation"].value
        reasoner = target_location._getReasoner("AauSpatialReasoner")
        reasoner.transform(target_location, "map")
        pose = target_location.getData(":PoseMsg")

        self.result = navigation.drive_to(pose)
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
