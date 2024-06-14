import pyrealsense2 as rs
import numpy as np
import cv2

import rospy
import rospkg
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge


# def pipeline_init():
rospy.init_node('rs_node', anonymous=True, disable_signals=False)
rate = rospy.Rate(10) # 10hz

bridge = CvBridge()

# We want the points object to be persistent so we can display the 
#last cloud when a frame drops
points = rs.points()
# Create a pipeline
pipeline = rs.pipeline()
#Create a config and configure the pipeline to stream
config = rs.config()
# config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.y8, 30)
# Start streaming
profile = pipeline.start(config)

#ROS Publisher
pub_cam = rospy.Publisher('mono_image', Image, queue_size = 10)
pub_imu = rospy.Publisher('imu0', Imu, queue_size = 1)
#ROS Subscriber
# sub_cam = rospy.Subscriber('mono_image', cam_cb(), queue_size = 1)

msg_imu = Imu()

def cam_cb(msg):
    print("[RS_NODE] Camera Time Stamp: ", msg.header.stamp)

def imu_cb(msg):
    print("[RS_NODE] IMU Time Stamp: ", msg.header.stamp)

def imu_construct():
    #Construct the  IMU message.
    msg = Imu()
    msg.header.stamp = rospy.get_rostime()

    #update IMU
    ax = 0.0
    ay = 0.0
    az = 0.0
    gx = 0.0
    gy = 0.0
    gz = 0.0
    q0 = 0.0 #W
    q1 = 0.0 #X
    q2 = 0.0 #Y
    q3 = 0.0 #Z

    #Fill message
    msg.orientation.x = q1
    msg.orientation.y = q2
    msg.orientation.z = q3
    msg.orientation.w = q0
    msg.orientation_covariance[0] = q1 * q1
    msg.orientation_covariance[0] = q2 * q2
    msg.orientation_covariance[0] = q3 * q3		

    msg.angular_velocity.x = gx
    msg.angular_velocity.y = gy
    msg.angular_velocity.z = gz
    msg.angular_velocity_covariance[0] = gx * gx
    msg.angular_velocity_covariance[4] = gy * gy
    msg.angular_velocity_covariance[8] = gz * gz
    
    msg.linear_acceleration.x = ax
    msg.linear_acceleration.y = ay
    msg.linear_acceleration.z = az
    msg.linear_acceleration_covariance[0] = ax * ax
    msg.linear_acceleration_covariance[4] = ay * ay
    msg.linear_acceleration_covariance[8] = az * az

    return msg


def stream_mono():
    # Streaming loop
    try:
        while not rospy.is_shutdown():
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # ir1_frame = frames.get_infrared_frame(1) # Left IR Camera, it allows 1, 2 or no input
            ir1_frame = frames.get_color_frame()
            image = np.asanyarray(ir1_frame.get_data())
            # cv2.namedWindow('IR Example', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('IR Example', image)

            msg_image = bridge.cv2_to_imgmsg(image, 'mono8')#encoding="passthrough")
            msg_image.header.stamp = rospy.get_rostime()
            msg_image.height = 720
            msg_image.width = 1280
            # msg_image.encoding = 'mono8'
            pub_cam.publish(msg_image)
            
            msg_imu = imu_construct()
            pub_imu.publish(msg_imu)

            # rospy.Subscriber('mono_image', Image, cam_cb, queue_size = 10)
            # rospy.Subscriber('imu0', Imu, imu_cb, queue_size = 1)

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()

def stopProcess():
  print ("[RS_NODE] Shutdown.")

rospy.on_shutdown(stopProcess)

if __name__ == '__main__':
    # pipeline_init()
    print("[RS_NODE] Started node.")
    try:
        stream_mono()
    except rospy.ROSInterruptException:
        pass