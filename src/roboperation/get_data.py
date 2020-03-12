import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

def string_to_pose(str_array, start_pose):
    desired_pose = Pose()
    # Position - PROPER POSITION
    desired_pose.position.x = start_pose.position.x + float(str_array[2])
    desired_pose.position.y = start_pose.position.y + float(str_array[3])
    desired_pose.position.z = start_pose.position.z + float(str_array[4])

    # desired_pose.position.x = start_pose.position.x
    # desired_pose.position.y = start_pose.position.y
    # desired_pose.position.z = start_pose.position.z

    # Orientation
    desired_pose.orientation.x = start_pose.orientation.x
    desired_pose.orientation.y = start_pose.orientation.y
    desired_pose.orientation.z = start_pose.orientation.z
    desired_pose.orientation.w = start_pose.orientation.w

    # desired_pose.orientation.x = float(str_array[5])
    # desired_pose.orientation.y = float(str_array[6])
    # desired_pose.orientation.z = float(str_array[7])
    # desired_pose.orientation.w = float(str_array[8])

    # desired_pose.orientation.x = start_pose.orientation.x + float(str_array[5])
    # desired_pose.orientation.y = start_pose.orientation.y + float(str_array[6])
    # desired_pose.orientation.z = start_pose.orientation.z + float(str_array[7])
    # desired_pose.orientation.w = start_pose.orientation.w + float(str_array[8])
    return desired_pose

def talker():
    gripper_state_publisher = rospy.Publisher('/roboperation_gripper/chatter', String, queue_size=10)
    desired_pose_stamped_publisher = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 10hz

    seq_num = 0
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "panda_link0"

    desired_pose = Pose()

    start_pose = Pose()
    start_pose.position.x = 0.308823701528
    start_pose.position.y = 0.000134250229
    start_pose.position.z = 0.48332632431
    start_pose.orientation.x = 0.999984212598
    start_pose.orientation.y = 0.000174034949
    start_pose.orientation.z = -0.005586324875
    start_pose.orientation.w = -0.000580725322

    open_state = "OPEN"
    close_state = "CLOSE"
    stop_state = "STOP"

    chatter_str = open_state

    while not rospy.is_shutdown():
        f = open("/home/robohub/Roboperation/src/roboperation/output.txt", 'r')
        content = f.read()
        rospy.loginfo(content)
        content.replace('\r\n', "")
        str_array = content.split(',')



        if len(str_array) == 11:
            gripper_state2 = str_array[10]

            print (str_array[10])
            if str_array[9].strip() == '1':
                chatter_str = open_state
            elif str_array[10].strip() == '1':
                chatter_str = close_state
            else:
                chatter_str = open_state

            print("Sending")
            print(chatter_str)

            pose_stamped.pose = string_to_pose(str_array, start_pose)
            gripper_state_publisher.publish(chatter_str)
            desired_pose_stamped_publisher.publish(pose_stamped)

        f.close()
        f = open("/home/robohub/Roboperation/src/roboperation/output.txt", 'w')
        f.write("---,")
        f.close()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
