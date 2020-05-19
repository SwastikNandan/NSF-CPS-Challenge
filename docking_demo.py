"""
    Harish
    2020-05-14
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
import math
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String, Header
import subprocess

class OffboardControl:
    """ Controller for PX4-UAV offboard mode """

    def __init__(self):
        self.curr_pose = PoseStamped()
        self.is_ready_to_fly = False
        self.hover_loc = [-2.29, -2.28, 3, 0, 0, 0, 0] # Hovers 3meter above at this location 
        self.mode = "HOVER"
        self.dist_threshold = 0.1
        self.arm = False

        # define ros subscribers and publishers
        rospy.init_node('OffboardControl', anonymous=True)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_callback)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.attach = rospy.Publisher('/attach', String, queue_size=10)
        self.decision = rospy.Subscriber('/data', String, callback=self.set_mode)
        self.controller()

    def set_mode(self, msg):
        self.mode = str(msg.data)

    def pose_callback(self, msg):
        self.curr_pose = msg

    def state_callback(self, msg):
        if msg.mode == 'OFFBOARD' and self.arm == True:
            self.is_ready_to_fly = True
        else:
            self.take_off()

    def set_offboard_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def set_arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armService(True)
            self.arm = True
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)

    def take_off(self):
        self.set_offboard_mode()
        self.set_arm()

    def hover(self):
        """ hover at height mentioned in location
            set mode as HOVER to make it work
        """
        location = self.hover_loc
        loc = [list(location),
               list(location),
               list(location),
               list(location),
               list(location),
               list(location),
               list(location)]

        # The setpoints are changed to reduce height of the drone
        loc[0][2] = 3
        loc[1][2] = 1
        loc[2][2] = 0.5
        loc[3][2] = 0.1
        loc[4][2] = 0.1
        loc[5][2] = 0.1
        loc[6][2] = 0.1

        rate = rospy.Rate(20)
        rate.sleep()
        shape = len(loc)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        des_pose = PoseStamped()
        waypoint_index = 0
        sim_ctr = 1
        print(self.mode)
        attach = False
        while self.mode == "HOVER" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1

                # Test detach 
                if not attach:
                    self.attach.publish("ATTACH")
                    attach = True
                rospy.sleep(0.2)

                # These points will make sure there is upward thrust on the drone after pickup
                location = self.hover_loc
                loc = [list(location),
                       list(location),
                       list(location),
                       list(location),
                       list(location),
                       list(location)]

                # Test detach 
                if sim_ctr == 15:
                    self.attach.publish("DETACH")
                
                print("HOVER COUNTER: " + str(sim_ctr))

            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            des_pose.pose.position.x = des_x
            des_pose.pose.position.y = des_y
            des_pose.pose.position.z = des_z
            des_pose.pose.orientation.x = loc[waypoint_index][3]
            des_pose.pose.orientation.y = loc[waypoint_index][4]
            des_pose.pose.orientation.z = loc[waypoint_index][5]
            des_pose.pose.orientation.w = loc[waypoint_index][6]

            curr_x = self.curr_pose.pose.position.x
            curr_y = self.curr_pose.pose.position.y
            curr_z = self.curr_pose.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) +
                             (curr_z - des_z)*(curr_z - des_z))
            if dist < self.dist_threshold:
                waypoint_index += 1

            pose_pub.publish(des_pose)
            rate.sleep()

    def controller(self):
        """ A state machine developed to have UAV states """
        while not rospy.is_shutdown():
            if self.mode == "HOVER":
                self.hover()


if __name__ == "__main__":
    OffboardControl()

