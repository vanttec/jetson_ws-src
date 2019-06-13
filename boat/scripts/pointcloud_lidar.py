from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math
import rospy


class Lidar:


    def __init__(self):
        self.points_list = []
        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback_zed_cp)

    def callback_zed_cp(self,ros_cloud):
        self.points_list = list(pc2.read_points(ros_cloud, skip_nans=False, field_names = ("x", "y", "z")))
        

        left,right,c = 0,0,0



        

        for i in len(self.points_list):
            if i[1] < -0.75:
                left += 1

            elif i[1] > 0.75:
                right += 1

            else:
                c += 1

        thresh = 10 #?

        l,r,c = left>thresh, right>thresh, center>thresh

        print(l,r,c)






if __name__ == '__main__':
    try:
        rospy.init_node('lidar_node')

        rate = rospy.Rate(10) # 10Hz
        D = Lidar()


        

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
