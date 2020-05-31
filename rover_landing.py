

def Rover(self):
	# assuming search is finished

	prev_x = 0
	prev_y = 0
	prev_z = 0

	while self.suv_detection >=1 and self.tag_detection < 10:

		x_goal = (self.curr_pose.pose.position.x + self.truck_target_x) + (self.truck_target_y-prev_x) # add x goal based on velocity of travel
		y_goal = (self.curr_pose.pose.position.y + self.truck_target_y) + (self.truck_target_y-prev_y) # add y goal based on velocity of travel
		z_goal = (self.curr_pose.pose.position.z + self.truck_target_z) + (self.truck_target_z- prev_z)
		location = [x_goal, y_goal, self.curr_pose.pose.position.z + (10-self.range),0,0,0,0] # z-value will maintain 10 m from ground

		flytodestination(location)

		prev_x = self.truck_target_x
		prev_y = self.truck_target_y
		prev_z = self.truck_target_z



	while self.tag_detection >= 10 and self.range > 0.7:

		tag_error_x = (self.destination_x-self.curr_pose.pose.position.x)
		tag_error_y = (self.destination_y-self.curr_pose.pose.position.x)

		x_goal = (self.curr_pose.pose.position.x + self.truck_target_x) + (self.truck_target_y-prev_x) + tag_error_x # add x goal based on velocity of travel
		y_goal = (self.curr_pose.pose.position.y + self.truck_target_y) + tag_error_y # add y goal based on velocity of travel


		err_x = (x_goal - self.curr_pose.pose.position.x)
		err_y = (y_goal - self.curr_pose.pose.position.x) 
       	
       	x_change = (err_x * self.KP * 50)#-(((err_x - self.x_prev_error) * 70) * self.KD)
        y_change = -(err_y * self.KP * 100)#-(((err_y - self.y_prev_error) * 70) * self.KD)    

        des = self.get_descent(x_change, y_change, -0.2)
        self. vel_pub.publish(des)

		prev_x = self.truck_target_x
		prev_y = self.truck_target_y




