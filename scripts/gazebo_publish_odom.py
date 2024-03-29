#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf

rospy.init_node('odom_pub')

odom_pub=rospy.Publisher('odom', Odometry, queue_size=1)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

br = tf.TransformBroadcaster()

odom=Odometry()
header = Header()
header.frame_id='/odom'

model = GetModelStateRequest()
model.model_name='cc_2019_car'

r = rospy.Rate(2)

while not rospy.is_shutdown():
    result = get_model_srv(model)

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish (odom)
    br.sendTransform((result.pose.position.x, result.pose.position.y, result.pose.position.z),
                 (result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w),
                 rospy.Time.now(),
                 "rear_axis_middle_ground",
                 "base_link")

    r.sleep()
