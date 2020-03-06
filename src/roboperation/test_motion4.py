import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

def talker():
    rospy.init_node('talker', anonymous=True)
    desired_pose_stamped_publisher = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(30) # Hz


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
