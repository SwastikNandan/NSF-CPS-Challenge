"""
    Harish
    2020-05-14
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
import math
import cv2
import time
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
import nav_msgs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, RegionOfInterest, Image
from image_geometry import PinholeCameraModel
from mavros_msgs.srv import SetMode, CommandBool
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Header
from sensor_msgs.msg import Range
import subprocess
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

class OffboardControl:

    """ Controller for PX4-UAV offboard mode """
    locations = np.matrix([[-1.8, -1.8, 2, 0, 0, 0, 0],
                            [-2.6, -1.8, 2, 0, 0, 0, 0],
                            [-2.6, -2.6, 2, 0,  0, 0, 0],
                            [-1.8, -2.6, 2, 0, 0, 0, 0],
                            ])
       
    search_loc = [80, -60, 20, 0, 0, 0, 0]
    drop_loc = [-75.998839, 425.005299, 30, 0, 0, 0, 0]

    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = 'uav' + str(uavID) + '/mavros/'
        return mav_topic_string


    def __init__(self):
        self.curr_pose = PoseStamped()
        self.curr_pose_rover = PoseStamped()
        self.cam_img = PoseStamped()
        self.des_pose = PoseStamped()
        self.destination_pose = PoseStamped()
        self.rover_variable = PositionTarget()
        self.pixel_img = Image()
        self.camera_image = Image()
        self.rover_location_x = 0
        self.rover_loaction_y = 0
        self.rover_location_z = 0
        self.rover_location_x_previous = 0
        self.rover_location_y_previous = 0
        self.rover_location_z_previous = 0
        self.truck_target_x = 0
        self.truck_target_y = 0
        self.truck_target_z = 0
        self.rover_velocity_x = 0
        self.rover_velocity_y = 0
        self.rover_velocity_z = 0
        self.rover_velocity_x_1 = 0
        self.rover_velocity_y_1 = 0
        self.rover_velocity_z_1 = 0
        self.rover_velocity_previous_x = 0
        self.rover_velocity_previous_y = 0
        self.rover_velocity_previous_z = 0
        self.image_target =[]
        self.depth = Image()
        self.KP= .005
        self.counter = 0
        self.destination_x = 0
        self.destination_y = 0
        self.destination_z = 0
        self.destination_x_previous = 0
        self.destination_y_previous = 0
        self.destination_z_previous = 0
        self.hover_x = 0
        self.hover_y = 0
        self.hover_z = 0
        self.detection_count = 0
        self.ray_target = []
        self.rel_coordinates = []
        self.flag = "False"
        self.is_ready_to_fly = False
        self.hover_loc = [self.hover_x, self.hover_y, self.hover_z, 0, 0, 0, 0] # Hovers 3meter above at this location 
        self.suv_search_loc = [0, 0, 0, 0, 0, 0, 0]
        self.suv_search_location = [0, 0, 0, 0, 0, 0, 0]
        self.mode = "FLYTODESTINATION"
        self.phase = "SEARCH"
        self.dist_threshold = 0.4
        self.waypointIndex = 0
        self.sim_ctr = 1
        self.arm = False
        self.range = 0
        self.prev_count = 0

        # define ros subscribers and publishers
        rospy.init_node('OffboardControl', anonymous=True)
        self.pose_sub = rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)
        self.pose_sub0 = rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, callback=self.pose_callback0)
        self.vel_sub0 = rospy.Subscriber('uav0/mavros/local_position/odom', Odometry, callback=self.velocity_callback0) 
        self.state_sub = rospy.Subscriber('uav1/mavros/state', State, callback=self.state_callback)
        self.vel_pub = rospy.Publisher('uav1/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        #self.attach = rospy.Publisher('/attach', String, queue_size=10)
        camera_info = rospy.wait_for_message("/uav_camera_down/camera_info", CameraInfo)
        camera_info2 = rospy.wait_for_message("/uav_camera_front/rgb/camera_info",CameraInfo)
        self.pinhole_camera_model = PinholeCameraModel()
        self.pinhole_camera_model_rgb = PinholeCameraModel()
        self.pinhole_camera_model.fromCameraInfo(camera_info)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback=self.yolo)
        rospy.Subscriber('/uav_camera_down/image_raw',Image,callback=self.pixel_image)
        self.decision = rospy.Subscriber('/data', String, callback=self.set_mode)    # doesnot appear immediately
        self.sonar = rospy.Subscriber('/uav_sonar', Range, callback=self.range_callback)

        NUM_UAV = 2
        mode_proxy = [None for i in range(NUM_UAV)]
        arm_proxy = [None for i in range(NUM_UAV)]

        for uavID in range(0, NUM_UAV):
            mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)


        # success = [None for i in range(NUM_UAV)]
        # for uavID in range(0, NUM_UAV):
        #     try:
        #         success[uavID] = mode_proxy[uavID](1, 'OFFBOARD')
        #     except rospy.ServiceException, e:
        #         print ("mavros/set_mode service call failed: %s" % e)

        # success = [None for i in range(NUM_UAV)]
        # for uavID in range(0, NUM_UAV):
        #     rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

        # for uavID in range(0, NUM_UAV):
        #     try:
        #         success[uavID] = arm_proxy[uavID](True)
        #     except rospy.ServiceException, e:
        #         print ("mavros1/set_mode service call failed: %s" % e)
        self.controller()

    def yolo(self,data):
        for a in data.bounding_boxes:
            
            if a.Class == "truck" or a.Class == "bus" or a.Class == "SUV" or a.Class =="kite":
                
                self.detection_count = self.detection_count + 1
                X = a.xmin + (a.xmax - a.xmin)/2
                Y = a.ymin + (a.ymax - a.ymin)/2
                #print(a.xmin)
                #print(X)
                self.image_target = list(self.pinhole_camera_model.projectPixelTo3dRay(( X, Y )))
                self.image_target[:] = [x/self.image_target[2] for x in self.image_target] 
                height = self.range

                self.truck_target_x1 = self.image_target[0]*height
                self.truck_target_y1 = self.image_target[1]*height
                self.truck_target_z1 = height

                relative_coordinates = np.array([[self.truck_target_x1],[self.truck_target_y1], [self.truck_target_z1]])
                hom_transformation = np.array([[0, 1, 0, 0],[1, 0, 0, 0],[ 0, 0, -1, 0],[0, 0, 0, 1]])
                homogeneous_coordinates = np.array([[relative_coordinates[0][0]],[relative_coordinates[1][0]],[relative_coordinates[2][0]],[1]])
                product =  np.matmul(hom_transformation,homogeneous_coordinates)
                self.truck_target_x = -product[0][0]
                self.truck_target_y = -product[1][0]
                self.truck_target_z = product[2][0]
                #print('X_coordinate_truck',self.truck_target_x)
                #print('Y_coordinate_truck',self.truck_target_y)
                #print('Z_coordinate_truck',self.truck_target_z)


    def pixel_image(self,img):
        self.camera_image = img
        try:
            self.pixel_img = CvBridge().imgmsg_to_cv2(self.camera_image, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        #image = cv2.cvtColor(self.pixel_img, cv2.COLOR_BGR2RGB)

        #clean image
        #image_blur = cv2.GaussianBlur(image,(3,3),0)
        image_blur = cv2.GaussianBlur(self.pixel_img,(3,3),0)
        #image_blur_hsv= cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)
        image_blur_hsv= cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)

        #filter by colour1
        min_red = np.array([120,40,80])
        max_red = np.array([190,230,255])
        mask1 = cv2.inRange(image_blur_hsv, min_red, max_red)

        #filter by colour2
        min_red2 = np.array([200, 40, 80])
        max_red2 = np.array([256, 230, 255])
        mask2 = cv2.inRange(image_blur_hsv, min_red2, max_red2)

        # Use the two masks to create double mask
        mask = mask1 + mask2

        edged = cv2.Canny(mask, 30, 200)

        contours,heirarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:] # Fix suggested on stack overflow
        cv2.drawContours(image_blur_hsv, contours, -1, (0, 255, 0), 3)
        cv2.imshow("Image_window",image_blur_hsv)
        cv2.waitKey(3)

        if np.size(contours)>=1:
            self.flag = "True"
            #print np.shape(contours[0])
            contour_area = [(cv2.contourArea(c)) for c in contours]
            contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
            biggest_contour = max(contour_sizes, key= lambda x: x[0])[1]

        elif np.size(contours)<1:
            self.flag = "False"

        if self.flag =="True":
            self.counter = self.counter + 1
            alpha1 = 0.5
            # Finding the area within the biggest contour and hence finding the centroid:
            area = cv2.contourArea(biggest_contour)
            M = cv2.moments(biggest_contour)
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            actualx = cx   #Pixel x value
            actualy = cy   #Pixel y value
            #print('Pixel X:', actualx)
            #print('Pixel Y:', actualy)
            self.ray_target = list(self.pinhole_camera_model.projectPixelTo3dRay((actualx,actualy)))

            self.ray_target[:] = [x/self.ray_target[2] for x in self.ray_target]   # rescaling the ray_target such that z is 1
            #self.ray_target[:] = [x/650 for x in self.ray_target] #159.88
            #print(self.ray_target)
            height = self.range
            x = self.ray_target[0]*height
            y = self.ray_target[1]*height
            z = height
            self.rel_coordinates = np.array([[x], [y], [z]])   # rel_coordinates stand for relative cordinates
            #print x
            #print y
            #print z
            hom_transformation = np.array([[0, 1, 0, 0],[1, 0, 0, 0],[ 0, 0, -1, 0],[0, 0, 0, 1]])
            homogeneous_coordinates = np.array([[self.rel_coordinates[0][0]],[self.rel_coordinates[1][0]],[self.rel_coordinates[2][0]],[1]])
            product =  np.matmul(hom_transformation,homogeneous_coordinates)
            self.cam_img.pose.position.x = -product[0][0]
            self.cam_img.pose.position.y = -product[1][0]
            self.cam_img.pose.position.z = product[2][0]
            #print('X',-product[0][0])
            #print('Y',-product[1][0])
            #print('Z',-product[2][0])
            self.cam_img.pose.orientation.x = 0
            self.cam_img.pose.orientation.y = 0
            self.cam_img.pose.orientation.z = 0
            self.cam_img.pose.orientation.w = 1
            self.destination_pose.pose.position.x = self.cam_img.pose.position.x + self.curr_pose.pose.position.x
            self.destination_pose.pose.position.y = self.cam_img.pose.position.y + self.curr_pose.pose.position.y
            self.destination_pose.pose.position.z = self.cam_img.pose.position.z + self.curr_pose.pose.position.z  
            self.destination_pose.pose.orientation.x = self.curr_pose.pose.orientation.x
            self.destination_pose.pose.orientation.y = self.curr_pose.pose.orientation.y
            self.destination_pose.pose.orientation.z = self.curr_pose.pose.orientation.z
            self.destination_pose.pose.orientation.w = self.curr_pose.pose.orientation.w
            self.destination_x = ((1 - alpha1)*self.destination_pose.pose.position.x) + (alpha1 * self.destination_x_previous)
            self.destination_y = ((1 - alpha1)*self.destination_pose.pose.position.y) +(alpha1 * self.destination_y_previous)
            self.destination_z = ((1 - alpha1)*self.destination_pose.pose.position.z) +(alpha1 * self.destination_z_previous)
            self.destination_x_previous = self.destination_x
            self.destination_y_previous = self.destination_y
            self.destination_z_previous = self.destination_z

    def set_mode(self, msg):
        #print('set_mode')
        self.mode = str(msg.data)

    def pose_callback(self, msg):
        #print('pose_callback')
        self.curr_pose = msg

    def pose_callback0(self,msg):

        self.curr_pose_rover = msg
        self.rover_location_x = self.curr_pose_rover.pose.position.x
        self.rover_location_y = self.curr_pose_rover.pose.position.y
        self.rover_location_z = self.curr_pose_rover.pose.position.z
        self.suv_search_loc = [self.rover_location_x, self.rover_location_y, self.rover_location_z, 0, 0, 0, 0]

        if self.prev_count == 0 or self.prev_count % 20 == 0:
            self.rover_location_x_previous = self.rover_location_x
            self.rover_location_y_previous = self.rover_location_y
            self.rover_location_z_previous = self.rover_location_z

        self.prev_count+= 1

        #print("Current", self.rover_location_x)
        #print("Past", self.rover_location_x_previous)

    def quaternion_to_yaw(self, w, x, y, z):
        #"""Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def velocity_callback0(self,velocity):
        beta = 0.7
        self.rover_variable = velocity
        #print("The rover variable is :", self.rover_variable )
        x_rotation = self.rover_variable.pose.pose.orientation.x
        y_rotation = self.rover_variable.pose.pose.orientation.y
        z_rotation = self.rover_variable.pose.pose.orientation.z
        w_rotation = self.rover_variable.pose.pose.orientation.w
        euler_z = self.quaternion_to_yaw(w_rotation, x_rotation, y_rotation, z_rotation)
        #print("The euler rotation Z", euler_z)
        z_net_rot = -euler_z
        #print("The net z rotation is :", z_net_rot)
        cos_angle = math.cos(z_net_rot)
        sin_angle = math.sin(z_net_rot)
        matrix_1 = np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]])
        matrix_2 = np.array([[cos_angle, -sin_angle, 0], [sin_angle, cos_angle, 0],[0, 0, 1]])
        matrix = np.matmul(matrix_1,matrix_2)
        hom_transformation = np.array([[matrix[0][0], matrix[0][1], matrix[0][2], 0],[matrix[1][0], matrix[1][1], matrix[1][2], 0],[matrix[2][0], matrix[2][1], matrix[2][2], 0],[0, 0, 0, 1]])
        #hom_transformation = np.array([[cos_angle, -sin_angle, 0, 0],[sin_angle, cos_angle, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
        #print("The homogeneous transformation matrix is :", hom_transformation)
        self.rover_velocity_x_1 = self.rover_variable.twist.twist.linear.x
        self.rover_velocity_y_1 = self.rover_variable.twist.twist.linear.y
        self.rover_velocity_z_1 = self.rover_variable.twist.twist.linear.z
        if self.rover_location_x-self.rover_location_x_previous < 0.001 and self.rover_location_x-self.rover_location_x_previous > -0.001 :
            self.rover_velocity_x_1 = 0
        if self.rover_location_y-self.rover_location_y_previous < 0.001 and self.rover_location_y-self.rover_location_y_previous > -0.001 :
            self.rover_velocity_y_1 = 0
        if self.rover_location_z-self.rover_location_z_previous < 0.0001 and self.rover_location_z-self.rover_location_z_previous > -0.0001  :
            self.rover_velocity_z_1 = 0
        hom_velocity = np.array([[self.rover_velocity_x_1],[self.rover_velocity_y_1],[self.rover_velocity_z_1],[1]])
        product =  np.matmul(hom_transformation,hom_velocity)
        self.rover_velocity_x = ((1 - beta)*product[0][0] + (beta)*self.rover_velocity_previous_x)
        self.rover_velocity_y = ((1 - beta)*product[1][0] + (beta)*self.rover_velocity_previous_y)
        self.rover_velocity_z = ((1 - beta)*product[2][0] + (beta)*self.rover_velocity_previous_z)
        self.rover_velocity_previous_x = self.rover_velocity_x
        self.rover_velocity_previous_y = self.rover_velocity_y
        self.rover_velocity_previous_z = self.rover_velocity_z
        #print("The X velocity of the ROVER",self.rover_velocity_x,"_________________________________________________")
        #print("The Y velocity of the ROVER",self.rover_velocity_y,"____________________________________________")
        #print("The Z velocity of the ROVER",self.rover_velocity_z,"____________________________________________________________")


    def range_callback(self,msg):
        #print(msg)
        self.range = msg.range

    def state_callback(self, msg):
        #print('state_callback')
        if msg.mode == 'OFFBOARD' and self.arm == True:
            self.is_ready_to_fly = True
        else:
            self.take_off()

    def set_offboard_mode(self):
        #print('set_offboardmode')
        rospy.wait_for_service('uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav1/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def set_arm(self):
        #print('set_arm')
        rospy.wait_for_service('uav1/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('uav1/mavros/cmd/arming', CommandBool)
            armService(True)
            self.arm = True
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)

    def take_off(self):
        #print('take_off')
        self.set_offboard_mode()
        self.set_arm()

    def attach(self):
        print('Running attach_srv')
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        attach_srv.wait_for_service()
        print('Trying to attach')

        req = AttachRequest()
        req.model_name_1 = "iris_1"
        req.link_name_1 = "base_link"
        req.model_name_2 = "sample_probe"
        req.link_name_2 = "base_link"

        attach_srv.call(req)
        print('Attached')

    def detach(self):
        attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
        attach_srv.wait_for_service()
        print("detaching")

        req = AttachRequest()
        req.model_name_1 = "iris_1"
        req.link_name_1 = "base_link"
        req.model_name_2 = "sample_probe"
        req.link_name_2 = "base_link"

        attach_srv.call(req)

    def hover(self):
        """ hover at height mentioned in location
        set mode as HOVER to make it work
        """
        location = self.hover_loc
        loc = [list(location),
        list(location),
        list(location),
        list(location),
        list(location)]
        print(loc)

        rate = rospy.Rate(20)
        rate.sleep()
        shape = len(loc)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose)
        waypoint_index = 0
        sim_ctr = 1
        #print(self.mode)
        
        while self.mode == "HOVER" and self.counter <= 35 and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("HOVER COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose.pose.position.x
            curr_y = self.curr_pose.pose.position.y
            curr_z = self.curr_pose.pose.position.z
            #print([curr_x, curr_y, curr_z])

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            #print(dist)
            if dist < self.dist_threshold:
                waypoint_index += 1

            pose_pub.publish(self.des_pose)
            rate.sleep()
        if self.counter > 35:
            self.mode = "SWOOP"

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose 

    def pattern(self):
        #print("PATTERN")
        self.sim_ctr = 0
        self.counter = 0
        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose)
        shape = self.locations.shape
        
        mower_ctr = 0
        x_increase = 0
        y_increase = 0
        z_increase = 0

        if self.phase == "SEARCH":
            des_x = self.search_loc[0] 
            des_y = self.search_loc[1]
            des_z = self.search_loc[2]

        if self.mode == "ROVER":
            self.sim_ctr = 0
            des_x = self.suv_search_loc[0]
            des_y = self.suv_search_loc[1]
            des_z = self.suv_search_loc[2]


        while not rospy.is_shutdown():
           
            if self.waypointIndex is shape[0]:
                #print("I am here")
                self.waypointIndex = 0
                self.sim_ctr += 1     


            if self.is_ready_to_fly : 


                self.des_pose.pose.position.x = des_x
                self.des_pose.pose.position.y = des_y
                self.des_pose.pose.position.z = des_z
                self.des_pose.pose.orientation.x = 0 
                self.des_pose.pose.orientation.y = 0
                self.des_pose.pose.orientation.z = 0 
                self.des_pose.pose.orientation.w = 0 
                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z
                x_increase = 0
                y_increase = 0
                z_increase = 0


                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
       

                if dist < self.dist_threshold:
                    self.waypointIndex += 1
    
                    if mower_ctr%4 == 0 or mower_ctr == 0:
                        x_increase += 0
                        y_increase += 10
                    if mower_ctr%2 == 0 and mower_ctr%4 != 0:
                        x_increase -= 0
                        y_increase -= 10
                    if mower_ctr%2 == 1:
                        x_increase += 3
                        y_increase += 0 

                    mower_ctr += 1

                    des_x = self.curr_pose.pose.position.x + x_increase 
                    des_y = self.curr_pose.pose.position.y + y_increase 

            
            z_increase = 5 - self.range # The integer value is chose such that the drone maintins the same value in units from the ground
            des_z = self.curr_pose.pose.position.z + z_increase 
            pose_pub.publish(self.des_pose)
            rate.sleep()
            # When it breaks here it should change to descend
            if self.counter >= 20:
                self.mode = "HOVER"
                self.hover_x = self.curr_pose.pose.position.x
                self.hover_y = self.curr_pose.pose.position.y
                self.hover_z = self.curr_pose.pose.position.z
                self.hover_loc = [self.hover_x, self.hover_y, self.hover_z, 0, 0, 0, 0]
                rate.sleep()
                break


    def flytodestination(self):
        print(self.phase)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        waypointidx = 0
        print("Waypointidx",waypointidx)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose)

        if self.phase == "SEARCH": #Phase set in the init function
            destination = np.matrix([self.search_loc,self.search_loc]) # Gives it a search location.

        if self.phase == "DROP":

            destination = np.matrix([[self.curr_pose.pose.position.x, self.curr_pose.pose.position.y, self.curr_pose.pose.position.z+15, 0, 0, 0, 0],
                                        self.drop_loc])

        while not rospy.is_shutdown():
            if waypointidx is 2:
                break
            if self.is_ready_to_fly:

                des_x = destination[waypointidx,0]
                des_y = destination[waypointidx,1]
                des_z = destination[waypointidx,2]
                #z_increase = 10 - self.range # The integer value is chose such that the drone maintins the same value in units from the ground
                #des_z = self.curr_pose.pose.position.z + z_increase 
                self.des_pose.pose.position.x = des_x
                self.des_pose.pose.position.y = des_y
                self.des_pose.pose.position.z = des_z
                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z
                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))

                if dist < self.dist_threshold:
                    waypointidx += 1 
                    print(waypointidx)

            pose_pub.publish(self.des_pose)
            rate.sleep()


        if self.phase == "SEARCH":
            self.mode = "PATTERN"

        if self.phase == "DROP":
            print("I am here in DROP")
            self.quickdrop()
            #self.descent1()
            #self.detach()
            #self.mode = "ROVER"


    def get_descent(self,x,y,z):
        #print("In get_desce   nt")
        des_vel = PositionTarget()
        des_vel.header.frame_id = "world"
        des_vel.header.stamp=rospy.Time.from_sec(time.time())
        des_vel.coordinate_frame= 8
        des_vel.type_mask = 3527
        des_vel.velocity.x = y #y
        des_vel.velocity.y = x #x
        des_vel.velocity.z = z

        return des_vel

    def quickdrop(self):
        self.sim_ctr = 0
        self.counter = 0
        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose)
        shape = self.locations.shape

        x_increase = 0
        y_increase = 0
        z_increase = 0
        des_x = -76
        des_y = 425 
        des_z = 30 
        
        while self.range > 1.25 and not rospy.is_shutdown():
           
            if self.waypointIndex is shape[0]:
                #print("I am here")
                self.waypointIndex = 0
                self.sim_ctr += 1     


            if self.is_ready_to_fly : 

                self.des_pose.pose.position.x = des_x
                self.des_pose.pose.position.y = des_y

                self.des_pose.pose.orientation.x = 0 
                self.des_pose.pose.orientation.y = 0
                self.des_pose.pose.orientation.z = 0 
                self.des_pose.pose.orientation.w = 0 
            
            z_increase = 1 - self.range # The integer value is chose such that the drone maintins the same value in units from the ground
            des_z = self.curr_pose.pose.position.z + z_increase 
            self.des_pose.pose.position.z = des_z
            pose_pub.publish(self.des_pose)
            rate.sleep()
            # When it breaks here it should change to descend
            # if self.counter >= 20:
            #     self.mode = "HOVER"
            #     self.hover_x = self.curr_pose.pose.position.x
            #     self.hover_y = self.curr_pose.pose.position.y
            #     self.hover_z = self.curr_pose.pose.position.z
            #     self.hover_loc = [self.hover_x, self.hover_y, self.hover_z, 0, 0, 0, 0]
            #     rate.sleep()
            #     break

        self.detach()
        self.mode = "ROVER"



    def descent2(self):
        print("In Descent2")
        rate = rospy.Rate(10)  # 10 Hz
        
        while self.range > 2 and not rospy.is_shutdown(): 

            err_x = self.truck_target_x - self.curr_pose.pose.position.x
            err_y = self.truck_target_y - self.curr_pose.pose.position.y
            err_z = self.truck_target_z - self.curr_pose.pose.position.z

            x_change = (err_x * self.KP * 50 + self.rover_velocity_x )
            print("rover velocity", self.rover_velocity_x )
            y_change = -(err_y * self.KP * 100 + self.rover_velocity_y )

            des = self.get_descent(x_change, y_change, -0.2)
            self.vel_pub.publish(des)
            self.x_prev_error = err_x
            self.y_prev_error = err_y
        self.descent3()

    def descent3(self):
        print("In Descent3")
        rate = rospy.Rate(10)  # 10 Hz
        
        while self.range > 0.5 and not rospy.is_shutdown(): 

            err_x = self.destination_x - self.curr_pose.pose.position.x
            err_y = self.destination_y - self.curr_pose.pose.position.y
            err_z = self.destination_z - self.curr_pose.pose.position.z

            x_change = (err_x * self.KP * 50 + self.rover_velocity_x )
            y_change = -(err_y * self.KP * 100 + self.rover_velocity_y )

            des = self.get_descent(x_change, y_change, -0.2)
            self.vel_pub.publish(des)
            self.x_prev_error = err_x
            self.y_prev_error = err_y
             


    def descent1(self):
        print("In Descent1")
        rate = rospy.Rate(10)  # 10 Hz
        
        while self.range > 1.8 and not rospy.is_shutdown(): 

            err_x = self.drop_loc[0] - self.curr_pose.pose.position.x
            err_y = self.drop_loc[1] - self.curr_pose.pose.position.y

            x_change = (err_x * self.KP * 50)
            y_change = -(err_y * self.KP * 100)

            des = self.get_descent(x_change, y_change, -0.2)
            self.vel_pub.publish(des)
            self.x_prev_error = err_x
            self.y_prev_error = err_y

            #rate.sleep()
        self.detach()
        self.mode = "ROVER"


    def descent(self):
        print("In Descent")
        rate = rospy.Rate(10)  # 10 Hz
        attach = False
        
        while self.mode == "SWOOP" and self.range > 0.7 and not rospy.is_shutdown():
            err_x = self.destination_x - self.curr_pose.pose.position.x
            err_y = self.destination_y - self.curr_pose.pose.position.y

            x_change = (err_x * self.KP * 50)
            y_change = -(err_y * self.KP * 100)

            des = self.get_descent(x_change, y_change, -0.2)
            self.vel_pub.publish(des)
            self.x_prev_error = err_x
            self.y_prev_error = err_y

            #rate.sleep()

        if self.phase == "SEARCH":
            self.attach() # The drone gets attached to the target
            self.phase = "DROP"
            self.mode = "FLYTODESTINATION"
            print("About to break out of descent to fly to destination")
        # if self.mode == "ROVER":
        #   print("End Here")
        #   self.mode = "END"

    def rover(self):
        #print("I am in rover")
        initial_location = self.drop_loc
        self.suv_search_location = [self.rover_location_x, self.rover_location_y, self.rover_location_z, 0, 0, 0, 0]
        location = self.suv_search_location
        loc = [list(initial_location),
               list(location)]
        print(loc)
        print(loc)
        print(loc)

        rate = rospy.Rate(10)
        #rate.sleep()
        shape = len(loc)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose)
        waypoint_index = 0
        sim_ctr = 1
        #print(self.mode)
        
        while self.mode == "ROVER" and sim_ctr<2 and not rospy.is_shutdown():
            #print("I am in while loop")
            if waypoint_index == 2:
                waypoint_index = 0
                sim_ctr += 1
            #print("HOVER COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            #des_z = loc[waypoint_index][2]
            z_increase = 20 - self.range # The integer value is chose such that the drone maintins the same value in units from the ground
            des_z = self.curr_pose.pose.position.z + z_increase 
            self.des_pose.pose.position.x = des_x
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose.pose.position.x
            curr_y = self.curr_pose.pose.position.y
            curr_z = self.curr_pose.pose.position.z
            #print([curr_x, curr_y, curr_z])

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            #print(dist)
            if dist < self.dist_threshold:
                waypoint_index += 1

            pose_pub.publish(self.des_pose)
            rate.sleep()
        self.square()

    def square(self):
        print("I am in square and I will do the zig-zag pattern")
        des_x = self.rover_location_x
        des_y = self.rover_location_y
        des_z = self.rover_location_z 
        # We gotta staert from here
        self.sim_ctr = 0
        self.counter = 0
        self.detection_count = 0
        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose)
        shape = self.locations.shape
        
        mower_ctr = 0
        x_increase = 0
        y_increase = 0
        z_increase = 0

        while not rospy.is_shutdown():
           
            if self.waypointIndex is shape[0]:
                print("I am here")
                self.waypointIndex = 0
                self.sim_ctr += 1     


            if self.is_ready_to_fly : 


                self.des_pose.pose.position.x = des_x
                self.des_pose.pose.position.y = des_y
                self.des_pose.pose.position.z = des_z
                self.des_pose.pose.orientation.x = 0 
                self.des_pose.pose.orientation.y = 0
                self.des_pose.pose.orientation.z = 0 
                self.des_pose.pose.orientation.w = 0 
                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z
                x_increase = 0
                y_increase = 0
                z_increase = 0


                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
       

                if dist < self.dist_threshold:
                    self.waypointIndex += 1
    
                    # if mower_ctr%2 == 0:
                    #     x_increase -= 10 # x change
                    # else:
                    #     x_increase += 10 # x change

                    # y_increase = 10

                    # mower_ctr += 1

                    # des_x = self.curr_pose.pose.position.x + x_increase 
                    # des_y = self.curr_pose.pose.position.y + y_increase 
                    des_x = self.rover_location_x
                    des_y = self.rover_location_y

            
            z_increase = 15 - self.range # The integer value is chose such that the drone maintins the same value in units from the ground
            des_z = self.curr_pose.pose.position.z + z_increase
            print("The new x, y, z of the rover is :", des_x, des_y, des_z) 
            pose_pub.publish(self.des_pose)
            rate.sleep()
            # When it breaks here it should change to descend
            if self.detection_count >= 4:                   # Not waiting too long for more detections as the rover moves out of sight
                self.descent2()
                rate.sleep()
                break


    def controller(self):

        print("Controller")
        while not rospy.is_shutdown():
            if self.mode == "PATTERN":
                self.pattern()
            if self.mode == "HOVER":
                self.hover()
            if self.mode == "SWOOP":
                self.descent()
            if self.mode == "FLYTODESTINATION":
                self.flytodestination()
            if self.mode == "ROVER":
                self.rover()

if __name__ == "__main__":
    OffboardControl()

