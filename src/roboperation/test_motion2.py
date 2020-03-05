import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

def talker():
    rospy.init_node('talker', anonymous=True)
    desired_pose_stamped_publisher = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(30) # Hz

    seq_num = 0
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "panda_link0"

    desired_pose = Pose()
    desired_pose.position.x = 0.532380569165
    desired_pose.position.y = -0.10012645809
    desired_pose.position.z = 0.425221838238
    desired_pose.orientation.x = 0.995123476557
    desired_pose.orientation.y = -0.0468952672936
    desired_pose.orientation.z =  0.0781914275355
    desired_pose.orientation.w = -0.037632445723

    positive_y_thresh = 0.20
    negative_y_thresh = -0.10
    rospy.loginfo("Publishing!!")

    while desired_pose.position.y < positive_y_thresh:
        desired_pose.position.y += 0.005
        desired_pose.orientation.y += 0.01
        desired_pose.orientation.w += 0.01
        rospy.loginfo("Pose(%f, %f, %f || %f, %f, %f, %f)", desired_pose.position.x, desired_pose.position.y, desired_pose.position.z, desired_pose.orientation.x, desired_pose.orientation.y, desired_pose.orientation.z, desired_pose.orientation.w)
        pose_stamped.header.seq = seq_num
        pose_stamped.pose = desired_pose
        desired_pose_stamped_publisher.publish(pose_stamped)
        seq_num = seq_num + 1
        rate.sleep()
    while desired_pose.position.y > negative_y_thresh:
        desired_pose.position.y -= 0.005
        desired_pose.orientation.y -= 0.01
        desired_pose.orientation.w -= 0.01
        rospy.loginfo("Pose(%f, %f, %f || %f, %f, %f, %f)", desired_pose.position.x, desired_pose.position.y, desired_pose.position.z, desired_pose.orientation.x, desired_pose.orientation.y, desired_pose.orientation.z, desired_pose.orientation.w)
        pose_stamped.header.seq = seq_num
        pose_stamped.pose = desired_pose
        desired_pose_stamped_publisher.publish(pose_stamped)
        seq_num = seq_num + 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
