#!/usr/bin/env python
import rospy
#from geopy.geocoders import utm
import numpy as np
import math
from pyproj import Proj
from numpy.linalg import inv
from sensor_msgs.msg import NavSatFix, Imu, CameraInfo, RegionOfInterest, Image
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from cps_challenge_2020.msg import gps_kf


class KalmanFilter:
    print("I am in offboard")
    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = 'uav' + str(uavID) + '/mavros/'
        return mav_topic_string

    def __init__(self):

        self.altitude_from_sealevel = 0
        self.UTMx = 0
        self.UTMy = 0
        self.height = 0
        self.rover_orientation_x = 0
        self.rover_orientation_y = 0
        self.rover_orientation_z = 0
        self.rover_linear_velocity = 0
        self.rover_angular_velocity = 0
        self.rover_linvel_x = 0
        self.rover_linvel_y = 0
        self.rover_linvel_z = 0
        self.rover_linvel_x_uncorrected = 0
        self.rover_linvel_y_uncorrected = 0
        self.rover_linvel_z_uncorrected = 0
        self.rover_angvel_x = 0
        self.rover_angvel_y = 0
        self.rover_angvel_z = 0
        self.rover_linacc_x = 0
        self.rover_linacc_y = 0
        self.rover_linacc_z = 0
        self.rover_angacc_x = 0
        self.rover_angacc_y = 0
        self.rover_angacc_z = 0
        self.timestep = 0.02       # Average frequency 50 HZ
        self.rover_roll = 0
        self.rover_pitch = 0
        self.rover_yaw = 0
        self.X_x = []
        self.X_y = []
        self.X_z = []
        self.count =0
        self.quaternion1 = []
        self.quaternion1_transpose = []
        self.quaternion2 = []
        rospy.init_node('KalmanFilter', anonymous=True)
        rospy.Subscriber('/uav0/mavros/global_position/raw/fix',NavSatFix, callback=self.navsat)
        rospy.Subscriber('/uav0/mavros/imu/data', Imu, callback=self.rover_imu)
        rospy.Subscriber('/odom',Odometry, callback=self.rover_odometry)
        self.pub = rospy.Publisher('kf_coords', gps_kf)
        self.KF()

    def rover_odometry(self,data):
         
        self.rover_linear_velocity = data.twist.twist.linear
        #print(self.rover_linear_velocity)
        self.rover_angular_velocity = data.twist.twist.angular
        self.quaternion2 = np.array([0, self.rover_linear_velocity.x, self.rover_linear_velocity.y, self.rover_linear_velocity.z])
        mul1 = self.quaternion_multiply(self.quaternion1,self.quaternion2)
        mul2 = self.quaternion_multiply(mul1,self.quaternion1_transpose)

        self.rover_linvel_x_uncorrected = mul2[1]
        self.rover_linvel_y_uncorrected = mul2[2]
        self.rover_linvel_z_uncorrected = mul2[3]
        rot_z_to_global = np.array([[0, 1, 0],[0, 0, 1],[1, 0, 0]])
        row_vector = np.array([self.rover_linvel_x_uncorrected, self.rover_linvel_y_uncorrected, self.rover_linvel_z_uncorrected])
        multiplication = np.matmul(rot_z_to_global,row_vector.T)
        self.rover_linvel_x = multiplication[0]
        self.rover_linvel_y = multiplication[1]
        self.rover_linvel_z = multiplication[2]
        #print("Corrected x velocity",self.rover_linvel_x)
        #print("Corrected y velocity",self.rover_linvel_y) 
        #self.T_base2world = self.tfROS.fromTranslationRotation((self.rover_linvel_x, self.rover_linvel_y, self.rover_linvel_z),(0, 0, 0, 0))
        #self.T_camera2world = np.matmul(self.T_base2world,self.T_camera2base)

    def navsat(self,data):
        #print(data)
        latitude = data.latitude
        longitude = data.longitude
        self.height = data.altitude
        #print(latitude, longitude)
        #UTM = utm.from_latlon(latitude,longitude)
        myProj = Proj("+proj=utm +zone=11S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
        self.UTMx, self.UTMy = myProj(longitude, latitude)
        #print("UTMx to check if the rover GPS is changing",self.UTMx)
        #print("UTMy to check if the rover GPS is changing",self.UTMy)

    def rover_imu(self,data):
        self.rover_orientation_x = data.orientation.x
        self.rover_orientation_y = data.orientation.y
        self.rover_orientation_z = data.orientation.z
        self.rover_orientation_w = data.orientation.w
        self.quaternion1 = np.array([self.rover_orientation_z, self.rover_orientation_w, self.rover_orientation_x, self.rover_orientation_y])
        self.quaternion1_transpose = np.array([self.rover_orientation_z, -self.rover_orientation_w, -self.rover_orientation_x, -self.rover_orientation_y])

        #orientation_list = [self.rover_orientation_x, self.rover_orientation_y, self.rover_orientation_z, self.rover_orientation_w]
        #(self.rover_roll, self.rover_pitch, self.rover_yaw) = euler_from_quaternion(orientation_list)
        self.rover_angvel_x = data.angular_velocity.x
        self.rover_angvel_y = data.angular_velocity.y
        self.rover_angvel_z = data.angular_velocity.z
        self.rover_linacc_x = data.linear_acceleration.x
        self.rover_linacc_y = data.linear_acceleration.y
        self.rover_linacc_z = data.linear_acceleration.z

    def quaternion_multiply(self,quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return array([-x1*x0 - y1*y0 - z1*z0 + w1*w0,
                     x1*w0 + y1*z0 - z1*y0 + w1*x0,
                    -x1*z0 + y1*w0 + z1*x0 + w1*y0,
                     x1*y0 - y1*x0 + z1*w0 + w1*z0], dtype=float64)


    def KF(self):

        def prediction2d(x, vx, ax, y, vy, ay, z, vz, az):
            #print("I am in prediction")
            self.count = self.count + 1
            A_x = np.array([[1,self.timestep],[0,1]])
            X_x = np.array([[x],[vx]])    
            B_x = np.array([[0.5*self.timestep**2],[self.timestep]])
            X_prime_x = A_x.dot(X_x) + B_x.dot(ax)

            A_y = np.array([[1,self.timestep],[0,1]])
            X_y = np.array([[y],[vy]])
            B_y = np.array([[0.5*self.timestep**2],[self.timestep]])
            X_prime_y = A_y.dot(X_y) + B_y.dot(ay)

            A_z = np.array([[1,self.timestep],[0,1]])
            X_z = np.array([[z],[vz]])
            B_z = np.array([[0.5*self.timestep**2],[self.timestep]])
            X_prime_z = A_y.dot(X_z) + B_y.dot(az)

            return X_prime_x, X_prime_y, X_prime_z

        def covariance2d(sigma1,sigma2):
            cov1_2 = sigma1*sigma2
            cov2_1 = sigma2*sigma1
            cov_matrix = np.array([[sigma1**2,cov1_2],[cov2_1,sigma2**2]])
            return np.diag(np.diag(cov_matrix))

        x = 0
        vx = self.rover_linvel_x
        ax = self.rover_linacc_x
        y = 0
        vy = self.rover_linvel_y
        ay = self.rover_linacc_y
        z = 0
        vz = self.rover_linvel_z
        az = self.rover_linacc_z
        error_est_x = 0.5
        error_est_vx = 0.5
        error_est_y = 0.5
        error_est_vy = 0.5
        error_est_z = 0.5
        error_est_vz = 0.5
        error_obs_x = 0.5
        error_obs_vx = 0.5
        error_obs_y = 0.5
        error_obs_vy = 0.5
        error_obs_z = 0.5
        error_obs_vz = 0.5
        msg = gps_kf()
        rate = rospy.Rate(50)
        P_x = covariance2d(error_est_x, error_est_vx)
        P_y = covariance2d(error_est_y, error_est_vy)
        P_z = covariance2d(error_est_z, error_est_vz)
        A_x = np.array([[1,self.timestep],[0,1]])
        A_y = np.array([[1,self.timestep],[0,1]])
        A_z = np.array([[1,self.timestep],[0,1]])
        self.X_x = np.array([[x],[vx]])
        #print(self.X_x)
        self.X_y = np.array([[y],[vy]])
        self.X_z = np.array([[z],[vz]])

        #self.X_x = np.squeeze(np.asarray(self.X_x))
        #self.X_y = np.squeeze(np.asarray(self.X_y))
        #self.X_z = np.squeeze(np.asarray(self.X_z))
        while not rospy.is_shutdown():
            if self.UTMx == 0:
                data_x = 0
            else:
                data_x = self.UTMx - (372232.32)

            if self.UTMy == 0:
                data_y = 0
            else:
                data_y = self.UTMy - 4146889.20
            data_z = self.height - 462.785
            ax = self.rover_linacc_x
            ay = self.rover_linacc_y
            az = self.rover_linacc_z
            #print(self.X_x)
            #self.X_x, self.X_y, self.X_z = prediction2d(self.X_x[0],self.X_x[1],ax,self.X_y[0],self.X_y[1],ay,self.X_z[0],self.X_z[1],az)            
            self.X_x, self.X_y, self.X_z = prediction2d(self.X_x[0][0],self.X_x[1][0],ax,self.X_y[0][0],self.X_y[1][0],ay,self.X_z[0][0],self.X_z[1][0],az)
            #print(self.X_x)
            P_x = np.diag(np.diag(A_x.dot(P_x).dot(A_x.T)))
            P_y = np.diag(np.diag(A_y.dot(P_y).dot(A_y.T)))
            P_z = np.diag(np.diag(A_z.dot(P_z).dot(A_z.T)))
            H = np.array([1,0])
            R_x = covariance2d(error_obs_x,error_obs_vx)
            R_y = covariance2d(error_obs_y,error_obs_vy)
            R_z = covariance2d(error_obs_z,error_obs_vz)
            #print(R_x)
            S_x = H.dot(P_x).dot(H.T) + R_x
            S_y = H.dot(P_y).dot(H.T) + R_y
            S_z = H.dot(P_z).dot(H.T) + R_z
            K_x = P_x.dot(H).dot(inv(S_x))
            #print(K_x)
            K_y = P_y.dot(H).dot(inv(S_y))
            K_z = P_z.dot(H).dot(inv(S_z))
            gps_array_x = np.array([data_x, 0])
            gps_array_y = np.array([data_y, 0])
            gps_array_z = np.array([data_z, 0])
            #print(data_x)
            Y_x = H.dot(data_x)
            Y_y = H.dot(data_y)
            Y_z = H.dot(data_z)
            #print(self.X_x.transpose())
            #self.X_x = np.squeeze(np.asarray(self.X_x))
            #self.X_y = np.squeeze(np.asarray(self.X_y))
            #self.X_z = np.squeeze(np.asarray(self.X_z))
            #print(self.X_x)
            M_x = H.dot(self.X_x)
            M_y = H.dot(self.X_y)
            M_z = H.dot(self.X_z)

            #print(Y_x)
            self.X_x = self.X_x + K_x.dot(Y_x - np.array([M_x[0],0]))
            #if self.count <= 10:
            #    print(self.UTMx)
            #    print("Y_x:",Y_x)
            #    print(self.X_x)
            #    print(H)
            #    print("M_x:", np.array([M_x[0],0]))
            #    print("Y_x - M_x:",Y_x - np.array([M_x[0],0]),"------------------------------")
            self.X_y = self.X_y + K_y.dot(Y_y - np.array([M_y[0],0]))
            self.X_z = self.X_z + K_z.dot(Y_z - np.array([M_y[0],0]))
            print(self.X_y)
            #P_x = (np.identity(len(K_x)) - K_x.dot(H)).dot(P_x)
            #P_y = (np.identity(len(K_y)) - K_y.dot(H)).dot(P_y)
            #P_z = (np.identity(len(K_z)) - K_z.dot(H)).dot(P_z)

            msg.X_x = self.X_x
            msg.X_y = self.X_y
            msg.X_z = self.X_z
            #rospy.loginfo(msg)
            self.pub.publish(msg)
            rate.sleep()


if __name__=="__main__":
    try:
        KalmanFilter()
    except rospy.ROSInterruptException: pass

















