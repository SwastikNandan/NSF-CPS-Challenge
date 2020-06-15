import rospy
#from geopy.geocoders import utm
import numpy as np
import math
from pyproj import Proj
from numpy.linalg import inv
from sensor_msgs.msg import NavSatFix, Imu, CameraInfo, RegionOfInterest, Image
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry


class OffboardControl:
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
        rospy.init_node('OffboardControl', anonymous=True)
        rospy.Subscriber('/uav1/mavros/global_position/raw/fix',NavSatFix, callback=self.navsat)
        rospy.Subscriber('/uav1/mavros/imu/data', Imu, callback=self.rover_imu)
        rospy.Subscriber('/odom',Odometry, callback=self.rover_odometry)
        self.KF()

    def rover_odometry(self,data):
         
        self.rover_linear_velocity = data.twist.twist.linear
        self.rover_angular_velocity = data.twist.twist.angular
        self.rover_linvel_x_uncorrected = self.rover_linear_velocity.x
        self.rover_linvel_y_uncorrected = self.rover_linear_velocity.y
        self.rover_linvel_z_uncorrected = self.rover_linear_velocity.z
        rot_z_to_global = np.array([[math.cos(-1.57), -math.sin(-1.57), 0],[math.sin(-1.57), math.cos(-1.57), 0], [0, 0, 1]])
        row_vector = np.array([self.rover_linvel_x, self.rover_linvel_y, self.rover_linvel_z])
        multiplication = np.matmul(rot_z_to_global,row_vector.T)
        self.rover_linvel_x = multiplication[0]
        self.rover_linvel_y = multiplication[1]
        self.rover_linvel_z = multiplication[2] 
        #self.T_base2world = self.tfROS.fromTranslationRotation((self.rover_linvel_x, self.rover_linvel_y, self.rover_linvel_z),(0, 0, 0, 0))
        #self.T_camera2world = np.matmul(self.T_base2world,self.T_camera2base)

    def navsat(self,data):
        #print(data)
        latitude = data.latitude
        longitude = data.longitude
        self.height = data.altitude
        #UTM = utm.from_latlon(latitude,longitude)
        myProj = Proj("+proj=utm +zone=23K, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
        self.UTMy, self.UTMx = myProj(longitude, latitude)
        #print(self.UTMx)
        #print(self.UTMy)

    def rover_imu(self,data):
        self.rover_orientation_x = data.orientation.x
        self.rover_orientation_y = data.orientation.y
        self.rover_orientation_z = data.orientation.z
        self.rover_orientation_w = data.orientation.w
        orientation_list = [self.rover_orientation_x, self.rover_orientation_y, self.rover_orientation_z, self.rover_orientation_w]
        (self.rover_roll, self.rover_pitch, self.rover_yaw) = euler_from_quaternion(orientation_list)
        self.rover_angvel_x = data.angular_velocity.x
        self.rover_angvel_y = data.angular_velocity.y
        self.rover_angvel_z = data.angular_velocity.z
        self.rover_linacc_x = data.linear_acceleration.x
        self.rover_linacc_y = data.linear_acceleration.y
        self.rover_linacc_z = data.linear_acceleration.z


    def KF(self):

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

        def prediction2d(x, vx, ax, y, vy, ay, z, vz, az):

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

        P_x = covariance2d(error_est_x, error_est_vx)
        P_y = covariance2d(error_est_y, error_est_vy)
        P_z = covariance2d(error_est_z, error_est_vz)
        A_x = np.array([[1,self.timestep],[0,1]])
        A_y = np.array([[1,self.timestep],[0,1]])
        A_z = np.array([[1,self.timestep],[0,1]])
        self.X_x = np.array([[x],[vx]])
        self.X_y = np.array([[y],[vy]])
        self.X_z = np.array([[z],[vz]])
        while not rospy.is_shutdown():
            data_x = self.UTMx - 7822938.26187
            data_y = (self.UTMy + 5927808.12211)*(-1)
            data_z = self.height - 462.785
            print("GPS x:", data_x, "GPS y:", data_y, "GPS z:", data_z)
            ax = self.rover_linacc_x
            ay = self.rover_linacc_y
            az = self.rover_linacc_z
            self.X_x, self.X_y, self.X_z = prediction2d(self.X_x[0][0],self.X_x[1][0],ax,self.X_y[0][0],self.X_y[1][0],ay,self.X_z[0][0],self.X_z[1][0],az)
            P_x = np.diag(np.diag(A_x.dot(P_x).dot(A_x.T)))
            P_y = np.diag(np.diag(A_y.dot(P_y).dot(A_y.T)))
            P_z = np.diag(np.diag(A_z.dot(P_z).dot(A_z.T)))
            H = np.array([1,0])
            #print(H)
            R_x = covariance2d(error_obs_x,error_obs_vx)
            R_y = covariance2d(error_obs_y,error_obs_vy)
            R_z = covariance2d(error_obs_z,error_obs_vz)
            S_x = H.dot(P_x).dot(H.T) + R_x
            S_y = H.dot(P_y).dot(H.T) + R_y
            S_z = H.dot(P_z).dot(H.T) + R_z
            K_x = P_x.dot(H).dot(inv(S_x))
            K_y = P_y.dot(H).dot(inv(S_y))
            K_z = P_z.dot(H).dot(inv(S_z))
            Y_x = H.dot(data_x)
            Y_y = H.dot(data_y)
            Y_z = H.dot(data_z)
            self.X_x = self.X_x + K_x.dot(Y_x - H.dot(self.X_x))
            self.X_y = self.X_y + K_y.dot(Y_y - H.dot(self.X_y))
            self.X_z = self.X_z + K_z.dot(Y_z - H.dot(self.X_z))
            #print(self.X_x)
            P_x = (np.identity(len(K_x)) - K_x.dot(H)).dot(P_x)
            P_y = (np.identity(len(K_y)) - K_y.dot(H)).dot(P_y)
            P_z = (np.identity(len(K_z)) - K_z.dot(H)).dot(P_z)
if __name__=="__main__":
    OffboardControl()

















