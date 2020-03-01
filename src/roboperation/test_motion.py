import rospy
from std_msgs.msg import String
from roboperation.msg import ArmStatePose
from geometry_msgs.msg import Pose

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    arm_state_pose_pub = rospy.Publisher('arm_control_input', ArmStatePose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5) # 10hz

    arm_state_pose_msg = ArmStatePose()
    arm_state_pose_msg.desired_pose.position.x = 99.0
    arm_state_pose_msg.desired_pose.position.y = 99.0
    arm_state_pose_msg.desired_pose.position.z = 99.0
    arm_state_pose_msg.desired_pose.orientation.x = 1.0
    arm_state_pose_msg.desired_pose.orientation.y = 2.0
    arm_state_pose_msg.desired_pose.orientation.z = 3.0

    # TODO: Need to change
    start_pose = Pose()
    start_pose.position.x = 200.0
    start_pose.position.y = 1.0
    start_pose.position.z = 1.0
    start_pose.orientation.x = 1.0
    start_pose.orientation.y = 1.0
    start_pose.orientation.z = 1.0

    test_points_length = 100
    test_points = []
    arm_state_pose_msg.desired_pose = start_pose
    test_points.append(arm_state_pose_msg)
    # for i in range(test_points_length):
    #     arm_state_pose_msg = ArmStatePose()
    #     arm_state_pose_msg.desired_pose = start_pose
    #     y_value = arm_state_pose_msg.desired_pose.position.y
    #     arm_state_pose_msg.desired_pose.position.y = y_value - 0.1
    #     rospy.loginfo("YValue %f", y_value)
    #     test_points.append(arm_state_pose_msg)


    # for point in test_points:
    # # while not rospy.is_shutdown():
    #     # hello_str = "Index %i" % index
    #     hello_str = "Heart beat %s" % rospy.get_time()
    #     rospy.loginfo("Y Value to Publish %f", point.desired_pose.position.y)
    #     pub.publish(hello_str)
    #
    #     arm_state_pose_pub.publish(point)
    #     # index = index + 1
    #     rate.sleep()

    end_state = 0.0
    offset = start_pose.position.x
    for i in range(test_points_length):
        test_pose = ArmStatePose()
        test_pose.desired_pose.position.x = offset + 0.1 * i
        rospy.loginfo("YValue %f", test_pose.desired_pose.position.x)
        arm_state_pose_pub.publish(test_pose)
        end_state = test_pose.desired_pose.position.x
        rate.sleep()

    for i in range(test_points_length):
        test_pose = ArmStatePose()
        test_pose.desired_pose.position.x = offset + end_state - (0.1 * i)
        rospy.loginfo("YValue %f", test_pose.desired_pose.position.x)
        arm_state_pose_pub.publish(test_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
