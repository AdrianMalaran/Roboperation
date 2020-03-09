import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

def string_to_pose(str_array, start_pose):
    desired_pose = Pose()
    desired_pose.position.x = start_pose.position.x + float(str_array[2])
    desired_pose.position.y = start_pose.position.y + float(str_array[3])
    desired_pose.position.z = start_pose.position.z + float(str_array[4])
    desired_pose.orientation.x = float(str_array[5])
    desired_pose.orientation.y = float(str_array[6])
    desired_pose.orientation.z = float(str_array[7])
    desired_pose.orientation.w = float(str_array[8])
    # desired_pose.orientation.x = start_pose.orientation.x + float(str_array[5])
    # desired_pose.orientation.y = start_pose.orientation.y + float(str_array[6])
    # desired_pose.orientation.z = start_pose.orientation.z + float(str_array[7])
    # desired_pose.orientation.w = start_pose.orientation.w + float(str_array[8])
    return desired_pose

def talker():
    gripper_state_publisher = rospy.Publisher('chatter', String, queue_size=10)
    desired_pose_stamped_publisher = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 10hz

    seq_num = 0
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "panda_link0"

    desired_pose = Pose()

    start_pose = Pose()
    start_pose.position.x = 0.532380569165
    start_pose.position.y = -0.10012645809
    start_pose.position.z = 0.425221838238
    start_pose.orientation.x = 0.995123476557
    start_pose.orientation.y = -0.0468952672936
    start_pose.orientation.z =  0.0781914275355
    start_pose.orientation.w = -0.037632445723

    # TESTING ORIENTATIONS
    # start_pose.orientation.x = -0.07698
    # start_pose.orientation.y = -0.12954
    # start_pose.orientation.z =  0.07959
    # start_pose.orientation.w = 0.98538

    open_state = "OPEN"
    close_state = "CLOSE"
    stop_state = "STOP"

    while not rospy.is_shutdown():
        f = open("/home/robohub/Roboperation/src/roboperation/output.txt", 'r')
        content = f.read()
        rospy.loginfo(content)
        content.replace('\r\n', "")
        str_array = content.split(',')

        chatter_str = stop_state

        if len(str_array) == 11:
            gripper_state2 = str_array[10] # Open Button

            if str_array[9] == "1":
                chatter_str = open_state
            elif str_array[10] == "1":
                chatter_str = close_state
            elif str_array[9] and str_array[10]:
                chatter_str = stop_state

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
