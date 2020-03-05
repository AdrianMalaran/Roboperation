import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def talker():
    rospy.init_node('talker', anonymous=True)
    desired_pose_publisher = rospy.Publisher('/input_state', Pose, queue_size=10)
    rate = rospy.Rate(30) # Hz

    desired_pose = Pose()
    desired_pose.position.x = 0.25
    desired_pose.position.y = 0.01
    desired_pose.position.z = 0.59
    desired_pose.orientation.w = 0.0013
    desired_pose.orientation.x = 0.9248
    desired_pose.orientation.y = -0.3806
    desired_pose.orientation.z =  0.0014
    desired_pose_publisher.publish(desired_pose)

    positive_y_thresh = 0.20
    negative_y_thresh = -0.10
    rospy.loginfo("Publishing!!")

    while desired_pose.position.y < positive_y_thresh:
        desired_pose.position.y += 0.001
        rospy.loginfo("Pose(%f, %f, %f || %f, %f, %f, %f)", desired_pose.position.x, desired_pose.position.y, desired_pose.position.z, desired_pose.orientation.x, desired_pose.orientation.y, desired_pose.orientation.z, desired_pose.orientation.w)
        desired_pose_publisher.publish(desired_pose)
        rate.sleep()
    while desired_pose.position.y > negative_y_thresh:
        desired_pose.position.y -= 0.001
        rospy.loginfo("Pose(%f, %f, %f || %f, %f, %f, %f)", desired_pose.position.x, desired_pose.position.y, desired_pose.position.z, desired_pose.orientation.x, desired_pose.orientation.y, desired_pose.orientation.z, desired_pose.orientation.w)
        desired_pose_publisher.publish(desired_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
