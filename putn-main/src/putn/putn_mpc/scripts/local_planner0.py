#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
import numpy as np
import tf
from MPC import MPC
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker,MarkerArray
from std_srvs.srv import SetBool


class Local_Planner():
    def __init__(self):
        self.replan_period = rospy.get_param('/local_planner/replan_period', 0.01)
        self.curr_state = np.zeros(5)
        self.z = 0
        self.N = 10
        self.goal_state = np.zeros([self.N,4])
        self.ref_path_close_set = False
        self.target_state = np.array([-1,4,np.pi/2])
        self.target_state_close = np.zeros(3)
        self.desired_global_path = [ np.zeros([300,4]) , 0]
        self.have_plan = False
        self.is_close = False
        self.is_get = False
        self.is_grasp = False
        self.is_all_task_down = False
        self.robot_state_set = False
        self.ref_path_set = False
        self.ob=[]
        self.is_end=0
        self.ob_total = []
        self.obt = []
        # self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)
        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb_no_crash)
        self.__sub_curr_state = rospy.Subscriber('curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=10)
        self.__sub_obs = rospy.Subscriber('obs', Float32MultiArray, self.__obs_cb, queue_size=10)
        self.__sub_goal_state = rospy.Subscriber('surf_predict_pub', Float32MultiArray, self._global_path_callback2, queue_size=10)

        self.__pub_local_path = rospy.Publisher('local_path', Path, queue_size=10)
        self.__pub_local_plan = rospy.Publisher('local_plan', Float32MultiArray, queue_size=10)
        self.__pub_zph_local_path = rospy.Publisher('local_path_zph', Path, queue_size=10)
        self.__pub_zph_local_plan = rospy.Publisher('local_plan_zph', Float32MultiArray, queue_size=10)
        self.__pub_zph_local_path_no_crash = rospy.Publisher('local_path_no_crash', Path, queue_size=10)
        self.control_cmd = Twist()
        self.listener = tf.TransformListener()
        self.times = 0
        self.obstacle_markerarray = MarkerArray()
        self.ob_pub = rospy.Publisher('ob_draw', MarkerArray, queue_size=10)
        self.no_obs_traj=[]

    def distance_sqaure(self,c1,c2):
        distance = (c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1])
        return distance

    def draw_ob(self):
        self.obstacle_markerarray.markers=[]
        num = 0
        # print("draw_ob")
        # print(self.ob)
        for i in range(len(self.ob)):
            t_ob = Marker()
            t_ob.header.frame_id = "world"
            t_ob.id = num
            t_ob.type = t_ob.CYLINDER
            t_ob.action = t_ob.ADD
            t_ob.pose.position.x = self.ob[i][0]
            t_ob.pose.position.y = self.ob[i][1]
            t_ob.pose.position.z=0.2
            t_ob.scale.x = 0.1
            t_ob.scale.y = 0.1
            t_ob.scale.z = 0.4
            t_ob.color.a= 1
            t_ob.color.r = 0
            t_ob.color.g = 1
            t_ob.color.b = 0
            self.obstacle_markerarray.markers.append(t_ob)
            num = num +1
        self.ob_pub.publish(self.obstacle_markerarray)

    def _scan_callback(self, data):
        self.ob = []
        phi = data.angle_min
        point_last = np.array([100, 100])
        for r in data.ranges:
            point = np.array([self.curr_state[0]+r*np.cos(phi+self.curr_state[2]),self.curr_state[1]+r*np.sin(phi+self.curr_state[2])])
            if (r >= data.range_min and r <= data.range_max and r<=1.0 and self.distance_sqaure(point,point_last) > 0.04 ):
                self.ob.append( point )
                point_last = point
            phi += data.angle_increment
        self.draw_ob()

    def __obs_cb(self, data):
        self.obt = []
        if(len(data.data)!=0):

            size = int(len(data.data)/3)
            for i in range(size):
                self.obt.append(( (data.data[3*i]//0.3)*0.3, (data.data[3*i+1]//0.3)*0.3) )
            dic = list(set([tuple(t) for t in self.obt]))
            self.obt = [list(v) for v in dic]
            self.ob = self.obt
            self.draw_ob()

    def __replan_cb(self, event):
        if self.robot_state_set and self.ref_path_set:
            target = []
            # self.choose_goal_state()        ##  gobal planning
            self.choose_goal_state_local_obs()
            dist = 1
            goal = np.array([self.target_state[0], self.target_state[1], self.target_state[2]])
            start_time = rospy.Time.now()
            # print('624')
            # print(self.goal_state)
            # print('624g')
            # print(goal)
            states_sol, input_sol = MPC(np.expand_dims(self.curr_state, axis=0),self.goal_state,self.ob) ##  gobal planning
            end_time = rospy.Time.now()
            rospy.loginfo('[pHRI Planner] phri so[lved in {} sec'.format((end_time-start_time).to_sec()))

            if(self.is_end == 0):
                self.__publish_local_plan(input_sol,states_sol)
            self.have_plan = True
        elif self.robot_state_set==False and self.ref_path_set==True:
            print("no pose")
        elif self.robot_state_set==True and self.ref_path_set==False:
            print("no path")
        else:
            print("no path and no pose")

    def __replan_cb_no_crash(self, event):
        if self.robot_state_set and self.ref_path_set:
            target = []
            # self.choose_goal_state()        ##  gobal planning
            self.choose_goal_state_local_obs()
            dist = 1
            goal = np.array([self.target_state[0], self.target_state[1], self.target_state[2]])
            start_time = rospy.Time.now()
            # print('624')
            # print(self.goal_state)
            # print('624g')
            # print(goal)
            if len(self.no_obs_traj)>0:
                print(np.array(self.no_obs_traj[0]))
                states_sol, input_sol = MPC(np.expand_dims(self.curr_state, axis=0),np.array(self.no_obs_traj[0]),self.ob) ##  gobal planning
                if(self.is_end == 0):
                    self.__publish_local_plan(input_sol,states_sol)
            else:
                print('error1')
                self.ref_path_set=False
            end_time = rospy.Time.now()
            rospy.loginfo('[pHRI Planner] phri so[lved in {} sec'.format((end_time-start_time).to_sec()))

            self.have_plan = True
        elif self.robot_state_set==False and self.ref_path_set==True:
            print("no pose")
        elif self.robot_state_set==True and self.ref_path_set==False:
            print("no path")
        else:
            print("no path and no pose")


    def __publish_local_plan(self,input_sol,state_sol):
        local_path = Path()
        local_plan = Float32MultiArray()
        sequ = 0
        local_path.header.stamp = rospy.Time.now()
        local_path.header.frame_id = "world"

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i,0]
            this_pose_stamped.pose.position.y = state_sol[i,1]
            this_pose_stamped.pose.position.z = self.z+0.5 #self.desired_global_path[0][0,2]
            this_pose_stamped.header.seq = sequ
            sequ += 1
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id="world"
            local_path.poses.append(this_pose_stamped)
            
            for j in range(2):
                local_plan.data.append(input_sol[i][j])

        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)

    def distance_global(self,c1,c2):
        distance = np.sqrt((c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1]))
        return distance
    

    def find_min_distance(self,c1):
        number =  np.argmin( np.array([self.distance_global(c1,self.desired_global_path[0][i]) for i in range(self.desired_global_path[1])]) )
        return number

    def choose_goal_state(self):
        num = self.find_min_distance(self.curr_state)
        scale = 1
        num_list = []
        for i in range(self.N):  
            num_path = min(self.desired_global_path[1]-1,int(num+i*scale))
            num_list.append(num_path)
        if(num  >= self.desired_global_path[1]):
            self.is_end = 1
        for k in range(self.N):
            self.goal_state[k] = self.desired_global_path[0][num_list[k]]
        # print(self.goal_state)

    def rotate_point(self,point, angle, origin):
        point = point - origin
        angle_rad = np.deg2rad(angle)
        rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                    [np.sin(angle_rad),  np.cos(angle_rad)]])
        
        rotated_point = np.dot(rotation_matrix, point)
        
        rotated_point += origin
        return rotated_point

    def is_trajectory_clear(self,trajectorys, obstacles):
        print('obstacles')
        print(obstacles)
        self.no_obs_traj=[]
        for trajectory in trajectorys:
            # print('trajectory')
            # print(trajectory)
            is_crash=False
            for point in trajectory:
                if is_crash:
                    break
                for obstacle in obstacles:

                    if np.linalg.norm(np.array(point[0:2])-np.array(obstacle[0:2]))<=1:
                        is_crash=True
            if not is_crash:
                self.no_obs_traj.append(trajectory)
        
        return True

    def local_path_generate(self):
        
        trajectory=np.array(self.goal_state)
        # print('trajectory')
        # print(trajectory)
        current_position = np.array(trajectory[0][:2])

        # direction_vector = np.array(trajectory[-1][:2]) - current_position0000000000000000000000000000000000000000000000000000000000000
        angles = [0,20,-20, 40, -40,60,-60,-80, 80,-100,100,-120,120,-140,140]
        rotated_trajectories = []

 
        for angle in angles:
            rotated_trajectory = []
            for point in trajectory:
                rotated_point = self.rotate_point(np.array(point[0:2]), angle, current_position)
                rotated_trajectory.append([rotated_point[0],rotated_point[1],0.5,0])
            rotated_trajectories.append(rotated_trajectory)

        ###############################################################
        local_path = Path()
        sequ = 0
        local_path.header.stamp = rospy.Time.now()
        local_path.header.frame_id = "world"
        for obs_path in rotated_trajectories:
            for points_ii in obs_path:
                this_pose_stamped = PoseStamped()
                this_pose_stamped.pose.position.x = points_ii[0]
                this_pose_stamped.pose.position.y = points_ii[1]
                this_pose_stamped.pose.position.z = 0.5 #self.desired_global_path[0][0,2]
                this_pose_stamped.header.seq = sequ
                sequ += 1
                this_pose_stamped.header.stamp = rospy.Time.now()
                this_pose_stamped.header.frame_id="world"
                local_path.poses.append(this_pose_stamped)
        ###############################################################
        # local_plan = Float32MultiArray()
        # for obs_path in rotated_trajectories:
        #     for points_ii in obs_path:
        #         local_plan.data.append(points_ii)
        # local_path = Path()
        self.__pub_zph_local_path.publish(local_path)
        print('ob')
        print(self.ob)
        # self.__pub_zph_local_plan.publish(local_plan)
        ########################################################################
        self.is_trajectory_clear(rotated_trajectories,self.ob)
        local_path_no_crash = Path()
        sequ = 0
        local_path_no_crash.header.stamp = rospy.Time.now()
        local_path_no_crash.header.frame_id = "world"
        for obs_path in self.no_obs_traj:
            for points_ii in obs_path:
                this_pose_stamped = PoseStamped()
                this_pose_stamped.pose.position.x = points_ii[0]
                this_pose_stamped.pose.position.y = points_ii[1]
                this_pose_stamped.pose.position.z = 0.5 #self.desired_global_path[0][0,2]
                this_pose_stamped.header.seq = sequ
                sequ += 1
                this_pose_stamped.header.stamp = rospy.Time.now()
                this_pose_stamped.header.frame_id="world"
                local_path_no_crash.poses.append(this_pose_stamped)
        self.__pub_zph_local_path_no_crash.publish(local_path_no_crash)
        # self.
        ########################################################################

    def choose_goal_state_local_obs(self):
        self.choose_goal_state()
        self.local_path_generate()
        # self.no_obs_traj
        # desired_global_path=self.no_obs_traj[0]
        # num = self.find_min_distance(self.curr_state)
        # scale = 1
        # num_list = []
        # for i in range(self.N):  
        #     num_path = min(self.desired_global_path[1]-1,int(num+i*scale))
        #     num_list.append(num_path)
        # if(num  >= self.desired_global_path[1]):
        #     self.is_end = 1
        # for k in range(self.N):
        #     self.goal_state[k] = self.desired_global_path[0][num_list[k]]
        # print(self.goal_state)
    

    def __curr_pose_cb(self, data):
        self.robot_state_set = True
        self.curr_state[0] = data.data[0]
        self.curr_state[1] = data.data[1]
        self.curr_state[2] = data.data[3]
        self.curr_state[3] = data.data[4]
        self.curr_state[4] = data.data[5]
 
        self.z = data.data[2]

    def _global_path_callback(self, data):
        if(len(data.data)!=0):
            print('cset1')
            self.ref_path_set = True
            size = len(data.data)/3
            self.desired_global_path[1]=size
            for i in range(size):
                self.desired_global_path[0][i,0]=data.data[3*(size-i)-3]
                self.desired_global_path[0][i,1]=data.data[3*(size-i)-2]
                self.desired_global_path[0][i,2]=data.data[3*(size-i)-1]
    
    def _global_path_callback2(self, data):
        if(len(data.data)!=0):
            print('set2')
            self.ref_path_set = True
            size = int(len(data.data)/5)
            self.desired_global_path[1]=size
            for i in range(size):
                self.desired_global_path[0][i,0]=data.data[5*(size-i)-5]
                self.desired_global_path[0][i,1]=data.data[5*(size-i)-4]
                self.desired_global_path[0][i,2]=data.data[5*(size-i)-2]
                self.desired_global_path[0][i,3]=data.data[5*(size-i)-1]
            
    def cmd(self, data):
        
        self.control_cmd.linear.x = data[0]
        self.control_cmd.angular.z = data[1]
        self.__pub_rtc_cmd.publish(self.control_cmd)



if __name__ == '__main__':
    rospy.init_node("local_planner")
    phri_planner = Local_Planner()

    rospy.spin()