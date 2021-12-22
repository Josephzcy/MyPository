import rospy
import rosbag
from sensor_msgs.msg import Imu
import pandas as pd

if __name__=='__main__':
  imu_data_path="/home/joseph/Mybag/imuData/2021-08-10-15-15-21.txt"
  imu_data= pd.read_csv(imu_data_path)
  print(imu_data)
  print(imu_data.columns.tolist())
  # print(imu_data[0])
  # a=imu_data[0][1]
  # print(imu_data.shape)
  cnt=0
  with rosbag.Bag('/home/joseph/Mybag/imuData/imu_raw_with_lidar.bag', 'w') as bag:
   
    for row in range(imu_data.shape[0]):
        timestamp = rospy.Time.from_sec(imu_data['Time'][row])
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp
        imu_msg.angular_velocity.x=imu_data['Gyro_X'][row]
        imu_msg.angular_velocity.y=imu_data['Gyro_Y'][row]
        imu_msg.angular_velocity.z=imu_data['Gyro_Z'][row]
        imu_msg.linear_acceleration.x=imu_data['Accel_X'][row]
        imu_msg.linear_acceleration.y=imu_data['Accel_Y'][row]
        imu_msg.linear_acceleration.z=imu_data['Accel_Z'][row]
        bag.write("/imu_raw", imu_msg, timestamp)
        cnt=cnt+1
        print(cnt)
        # Populate the data elements for IMU
        # e.g. imu_msg.angular_velocity.x = df['a_v_x'][row]
    
       



     

