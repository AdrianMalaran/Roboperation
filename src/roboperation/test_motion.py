import rospy
from std_msgs.msg import String
from roboperation.msg import ArmStatePose
from geometry_msgs.msg import Pose

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    arm_state_pose_pub = rospy.Publisher('arm_control_input', ArmStatePose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5) # 10hz

    arm_state_pose_pub.publish()



    # TODO: Need to change
    start_pose = Pose()
    start_pose.position.x = 0.25
    start_pose.position.y = 0.01
    start_pose.position.z = 0.59
    start_pose.orientation.w = 0.0013
    start_pose.orientation.x = 0.9248
    start_pose.orientation.y = -0.3806
    start_pose.orientation.z =  0.0014

    # Reset to Home
    home_state_pose = ArmStatePose()
    home_state_pose.desired_pose = start_pose
    arm_state_pose_pub.publish(home_state_pose)

    rospy.sleep(3.)

    test_points_length = 50
    end_state = 0.0
    offset = start_pose.position.x
    for i in range(test_points_length):
        test_pose = ArmStatePose()
        test_pose.desired_pose = start_pose
        test_pose.desired_pose.position.x = offset + 0.001 * i
        rospy.loginfo("YValue %f", test_pose.desired_pose.position.x)
        arm_state_pose_pub.publish(test_pose)
        end_state = test_pose.desired_pose.position.x
        rate.sleep()

    for i in range(test_points_length):
        test_pose = ArmStatePose()
        test_pose.desired_pose = start_pose
        test_pose.desired_pose.position.x = end_state - (0.001 * i)
        rospy.loginfo("YValue - Returning %f, %f, %f", test_pose.desired_pose.position.x, test_pose.desired_pose.position.y, test_pose.desired_pose.position.z)
        arm_state_pose_pub.publish(test_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
