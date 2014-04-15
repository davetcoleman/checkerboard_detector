#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest("posedetection_msgs")
#roslib.load_manifest("dynamic_tf_publisher")
from posedetection_msgs.msg import ObjectDetection
#from dynamic_tf_publisher.srv import *
import tf

def callback(msg):
    global object_messages
    object_messages = msg

if __name__== '__main__':
    global object_messages
    object_messages = None

    rospy.init_node('objectdetection_tf_publisher', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo("Starting object detection tf publisher")

    subscriber = rospy.Subscriber("ObjectDetection", ObjectDetection, callback)
    r = rospy.Rate(100)
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        if object_messages:
            for detected_object in object_messages.objects:
                br.sendTransform((detected_object.pose.position.x,
                                  detected_object.pose.position.y,
                                  detected_object.pose.position.z),
                                 (detected_object.pose.orientation.x,
                                  detected_object.pose.orientation.y,
                                  detected_object.pose.orientation.z,
                                  detected_object.pose.orientation.w),
                                 rospy.Time.now(),
                                 # child
                                 "/object",
                                 # parent
                                 object_messages.header.frame_id)
        r.sleep()

