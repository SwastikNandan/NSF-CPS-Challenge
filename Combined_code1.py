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
from sensor_msgs.msg import CameraInfo, RegionOfInterest, Image
from image_geometry import PinholeCameraModel
from mavros_msgs.srv import SetMode, CommandBool
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Header
import subprocess
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class OffboardControl:
    
    """ Controller for PX4-UAV offboard mode """
    locations = np.matrix([[-1.8, -1.8, 2, 0, 0, 0, 0],
                              [-2.6, -1.8, 2, 0, 0, 0, 0],
                              [-2.6, -2.6, 2, 0,  0, 0, 0],
                              [-1.8, -2.6, 2, 0, 0, 0, 0],
                              ])

    #locations = np.matrix([[-2.2, -2.2, 2, 0, 0, 0, 0],
     #                         [-2.2, -2.2, 2, 0, 0, 0, 0],
      #                        [-2.2, -2.2, 2, 0,  0, 0, 0],
       #                       [-2.2, -2.2, 2, 0, 0, 0, 0],
        #                      ])
    #locations = np.matrix([[-1.81, -2.13, 2, 0, 0, 0, 0],
     #                         [-1.85, -2.0, 2, 0, 0, 0, 0],
      #                        [-2.0, -1.85, 2, 0,  0, 0, 0],
       #                       [-2.13, -1.81, 2, 0, 0, 0, 0],
    	#		[-2.20, -1.80, 2, 0, 0, 0, 0],
    	#		[-2.27, -1.81, 2, 0, 0, 0, 0],
    	#		[-2.40, -1.85, 2, 0, 0, 0, 0],
	###		[-2.55, -2.00, 2, 0, 0, 0, 0],
	#		[-2.59, -2.13, 2, 0, 0, 0, 0],
	#		[-2.60, -2.20, 2, 0, 0, 0, 0],
	#		[-2.59, -2.27, 2, 0, 0, 0, 0],
	#		[-2.55, -2.40, 2, 0, 0, 0, 0],
	#		[-2.40, -2.55, 2, 0, 0, 0, 0],
	#		[-2.27, -2.59, 2, 0, 0, 0, 0],
	#		[-2.2, -2.60, 2, 0, 0, 0, 0],
	#		[-2.13, -2.59, 2, 0, 0, 0, 0],
	#		[-2.00, -2.55, 2, 0, 0, 0, 0],
	#		[-1.85, -2.40, 2, 0, 0, 0, 0],
	#		[-1.81, -2.27, 2, 0, 0, 0, 0],
	#		[-1.80, -2.20, 2, 0, 0, 0, 0]
         #                     ])
    
    def __init__(self):
        self.curr_pose = PoseStamped()
	self.cam_img = PoseStamped()
	self.des_pose = PoseStamped()
	self.destination_pose = PoseStamped()
	self.pixel_img = Image()
	self.KP= .005
	self.counter = 0
	self.destination_x = 0
	self.destination_y = 0
	self.destination_z = 0
	self.destination_x_previous = 0
        self.destination_y_previous = 0
        self.destination_z_previous = 0
	self.ray_target = []
	self.rel_coordinates = []
	self.flag = "False"
        self.is_ready_to_fly = False
        self.hover_loc = [self.destination_x, self.destination_y, 2, 0, 0, 0, 0] # Hovers 3meter above at this location 
        self.mode = "PATTERN"
        self.dist_threshold = 0.2
	self.waypointIndex = 0
	self.sim_ctr = 1
        self.arm = False

        # define ros subscribers and publishers
        rospy.init_node('OffboardControl', anonymous=True)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_callback)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        #self.attach = rospy.Publisher('/attach', String, queue_size=10)
        camera_info = rospy.wait_for_message("/uav_camera/camera_info", CameraInfo)
        self.pinhole_camera_model = PinholeCameraModel()
	self.pinhole_camera_model.fromCameraInfo(camera_info)
	#rospy.Subscriber('/uav_camera/image_raw_down',Image,callback=self.pixel_image, callback_args = '/uav_camera/image_raw_down')
	rospy.Subscriber('/uav_camera/image_raw_down',Image,callback=self.pixel_image)
        self.decision = rospy.Subscriber('/data', String, callback=self.set_mode)
        self.controller()

    def pixel_image(self,img):
	try:
	    self.pixel_img = CvBridge().imgmsg_to_cv2(img, desired_encoding='passthrough')
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
	    self.ray_target[:] = [x/650 for x in self.ray_target] #159.88
	    #print(self.ray_target)
	    height = self.curr_pose.pose.position.z + 0.15
	    x = self.ray_target[0]*height
	    y = self.ray_target[1]*height
	    z = height
	    self.rel_coordinates = np.array([[x], [y], [z]])   # rel_coordinates stand for relative cordinates
	    #print x
	    #print y
	    #print z
	    hom_transformation = np.array([[0, 1, 0, -0.12],[1, 0, 0, 0],[ 0, 0, -1, 0.01],[0, 0, 0, 1]])
	    homogeneous_coordinates = np.array([[self.rel_coordinates[0][0]],[self.rel_coordinates[1][0]],[self.rel_coordinates[2][0]],[1]])
	    product =  np.matmul(hom_transformation,homogeneous_coordinates)
	    self.cam_img.pose.position.x = product[0][0]
            self.cam_img.pose.position.y = product[1][0]
            self.cam_img.pose.position.z = product[2][0]
	    #print(product[0][0])
	    #print(product[1][0])
	    #print(-product[2][0])
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
	    #print("X target:",self.destination_x)
	    #print("Y target:",self.destination_y)
	    #print("Z target:",self.destination_z)
	     

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

    def attach(self):
	print('Running attach_srv')
	attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
	attach_srv.wait_for_service()
	print('Trying to attach')

	req = AttachRequest()
	req.model_name_1 = "iris"
	req.link_name_1 = "base_link"
	req.model_name_2 = "sample_probe"
	req.link_name_2 = "base_link"

	attach_srv.call(req)
	print('Attached')

    def detach(self):
	attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
	attach_srv.wait_for_service()
	
	req = AttachRequest()
	req.model_name_1 = "iris"
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
               list(location),
               list(location),
               list(location),
	       list(location),
	       list(location),
	       list(location),
               list(location)]
        # The setpoints are changed to reduce height of the drone
        loc[0][2] = 2
        loc[1][2] = 0.5
        loc[2][2] = 0.1
        loc[3][2] = 0.1
        loc[4][2] = 0.1
        loc[5][2] = 0.1
        loc[6][2] = 0.0
	loc[7][2] = 0.0
	loc[8][2] = 0.0
	loc[9][2] = 0.0
	loc[10][2] = 0.0
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
                #if sim_ctr == 15:
                #    self.attach.publish("DETACH")
                
                #print("HOVER COUNTER: " + str(sim_ctr))

            if attach == True:
	        self.mode = "FLYTODESTINATION"
		print("About to break the loop")
		break

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

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose 

    def pattern(self):

	rate = rospy.Rate(10)  # Hz
        rate.sleep()
	pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose)
        shape = self.locations.shape
	while not rospy.is_shutdown():
            #print self.sim_ctr, shape[0], self.waypointIndex
            if self.waypointIndex is shape[0]:
                self.waypointIndex = 0
                self.sim_ctr += 1

            if self.is_ready_to_fly:

                des_x = self.locations[self.waypointIndex, 0]
                des_y = self.locations[self.waypointIndex, 1]
                des_z = self.locations[self.waypointIndex, 2]
                self.des_pose.pose.position.x = des_x
                self.des_pose.pose.position.y = des_y
                self.des_pose.pose.position.z = des_z
                self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
                self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
                self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
                self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]


                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z

                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
                if dist < self.dist_threshold:
                    self.waypointIndex += 1

                # print dist, curr_x, curr_y, curr_z, self.waypointIndex
            pose_pub.publish(self.des_pose)
	    if self.counter >= 200:
		self.mode = "SWOOP"
	        break
            rate.sleep()


    def flytodestination(self):
	rate = rospy.Rate(10)  # Hz
        rate.sleep()
	waypointidx = 0
	pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose)
	destination = np.matrix([[25, -35, 10, 0 , 0, 0, 0],
				  [25, -35, 10, 0 , 0, 0, 0],
			          [25, -35, 2, 0 , 0, 0, 0],
	                          [25, -35, 2, 0 , 0, 0, 0]
                                  ])
	shape = destination.shape
        while not rospy.is_shutdown():
	    if waypointidx is 4:
	        break
	    if self.is_ready_to_fly:
                des_x = destination[waypointidx,0]
                des_y = destination[waypointidx,1]
                des_z = destination[waypointidx,2]
	        self.des_pose.pose.position.x = des_x
                self.des_pose.pose.position.y = des_y
                self.des_pose.pose.position.z = des_z
	        curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z
	        dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
		if dist < self.dist_threshold:
                    waypointidx += 1            
            pose_pub.publish(self.des_pose)
	    rate.sleep()
	self.detach()
        #self.attach.publish("DETACH")
	self.mode = "END"

    def get_descent(self,x,y,z):
	#print("In get_descent")
        des_vel = PositionTarget()
	des_vel.header.frame_id = "world"
	des_vel.header.stamp=rospy.Time.from_sec(time.time())
        des_vel.coordinate_frame= 8
	des_vel.type_mask = 3527
	des_vel.velocity.x = x #y
	des_vel.velocity.y = y #x
	des_vel.velocity.z = z
		
	return des_vel



    def descent(self):
	print("In Descent")
        rate = rospy.Rate(10)  # 10 Hz
	attach = False
        while self.mode == "SWOOP" and self.curr_pose.pose.position.z > 0.1 and not rospy.is_shutdown():
            #print("X target:",self.destination_x)
	    #print("Y target:",self.destination_y)
            err_x = self.destination_x - self.curr_pose.pose.position.x
            err_y = self.destination_y - self.curr_pose.pose.position.y

	    x_change = (self.rel_coordinates[0][0] * self.KP * 19)#-(((err_x - self.x_prev_error) * 70) * self.KD)
            y_change = -(self.rel_coordinates[1][0] * self.KP * 19)#-(((err_y - self.y_prev_error) * 70) * self.KD)	
            
	    #x_change = -(err_x * self.KP * 50)#-(((err_x - self.x_prev_error) * 70) * self.KD)
            #y_change = (err_y * self.KP * 100)#-(((err_y - self.y_prev_error) * 70) * self.KD)
	    
            des = self.get_descent(x_change, y_change, -0.1)
            self.vel_pub.publish(des)
	    self.x_prev_error = err_x
            self.y_prev_error = err_y
           
            rate.sleep()

	self.attach() # The drone gets attached to the target
	attach = True
	if attach == False:
	    print("In no attach")
	    attach = True
            self.attach.publish("ATTACH")
            rospy.sleep(0.2)
        if attach == True:
	    self.mode = "FLYTODESTINATION"
            print("About to break out of descent to fly to destination")
	    


    def controller(self):
        """ A state machine developed to have UAV states """
        while not rospy.is_shutdown():
	    if self.mode == "PATTERN":
	        self.pattern()
            if self.mode == "SWOOP":
                self.descent()
	    if self.mode == "FLYTODESTINATION":
	        self.flytodestination()


if __name__ == "__main__":
    OffboardControl()

