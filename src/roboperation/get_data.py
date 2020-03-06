import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

def string_to_pose(str_array):
    desired_pose = Pose()
    desired_pose.position.x = float(str_array[2])
    desired_pose.position.y = float(str_array[3])
    desired_pose.position.z = float(str_array[4])
    desired_pose.orientation.x = float(str_array[5])
    desired_pose.orientation.y = float(str_array[6])
    desired_pose.orientation.z = float(str_array[7])
    desired_pose.orientation.w = float(str_array[8])
    return desired_pose

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    desired_pose_stamped_publisher = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 10hz

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

    while not rospy.is_shutdown():
        f = open("/home/robohub/Roboperation/src/roboperation/output.txt", 'r')
        content = f.read()
        rospy.loginfo(content)
        content.replace('\r\n', "")
        str_array = content.split(',')
        if len(str_array) == 9:
            pose_stamped.pose = string_to_pose(str_array)
            pub.publish(content)
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
