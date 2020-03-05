import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from visualization_msgs.msg import  InteractiveMarker, InteractiveMarkerUpdate, InteractiveMarkerPose, InteractiveMarkerInit, InteractiveMarkerControl

def talker():
    rospy.init_node('talker', anonymous=True)
    # desired_pose_publisher = rospy.Publisher('/equilibrium_pose', Pose, queue_size=10)
    interactive_marker_init_publisher = rospy.Publisher('/equilibrium_pose_marker/update_full', InteractiveMarkerInit, queue_size=10)
    rate = rospy.Rate(30) # Hz

    seq_num = 1693
    # interactive_marker.type = 1 # MAY NEED TO UNCOMMENT IF IT DOESNT WORK

    # Topic to be published
    interactive_marker_init = InteractiveMarkerInit()
    interactive_marker_init.server_id  = "/interactive_marker"

    interactive_marker = InteractiveMarker()

    interactive_marker.name = "equilibrium_pose"
    interactive_marker.header.frame_id = "panda_link0"
    interactive_marker.scale = 0.3
    interactive_marker.description = "Equilibrium Pose\nBE CAREFUL! If you move the \nequilibrium pose the robot will follow\
  \ it\nso be aware of potential collisions"

    # Controls - X direction
    imc = InteractiveMarkerControl()
    imc.name = "rotate_x"
    imc.orientation.x = 1.0
    imc.orientation.y = 0
    imc.orientation.z = 0
    imc.orientation.w = 1.0
    imc.orientation_mode = 0
    imc.interaction_mode = 5
    interactive_marker.controls.append(imc)

    imc = InteractiveMarkerControl()
    imc.name = "move_x"
    imc.orientation.x = 1.0
    imc.orientation.y = 0
    imc.orientation.z = 0
    imc.orientation.w = 1.0
    imc.orientation_mode = 0
    imc.interaction_mode = 3
    interactive_marker.controls.append(imc)

    # Controls - Y direction
    imc = InteractiveMarkerControl()
    imc.name = "rotate_y"
    imc.orientation.x = 0
    imc.orientation.y = 1.0
    imc.orientation.z = 0
    imc.orientation.w = 1.0
    imc.orientation_mode = 0
    imc.interaction_mode = 5
    interactive_marker.controls.append(imc)

    imc = InteractiveMarkerControl()
    imc.name = "move_y"
    imc.orientation.x = 0
    imc.orientation.y = 1.0
    imc.orientation.z = 0
    imc.orientation.w = 1.0
    imc.orientation_mode = 0
    imc.interaction_mode = 3
    interactive_marker.controls.append(imc)

    # Controls - Z direction
    imc = InteractiveMarkerControl()
    imc.name = "rotate_z"
    imc.orientation.x = 0
    imc.orientation.y = 0
    imc.orientation.z = 1.0
    imc.orientation.w = 1.0
    imc.orientation_mode = 0
    imc.interaction_mode = 5
    interactive_marker.controls.append(imc)

    imc = InteractiveMarkerControl()
    imc.name = "move_z"
    imc.orientation.x = 0
    imc.orientation.y = 0
    imc.orientation.z = 1.0
    imc.orientation.w = 1.0
    imc.orientation_mode = 0
    imc.interaction_mode = 3
    interactive_marker.controls.append(imc)

    desired_pose = Pose()
    desired_pose.position.x = 0.441047421421
    desired_pose.position.y = 0.00039597222295
    desired_pose.position.z = 0.428872718226
    desired_pose.orientation.x = 0.998920857906
    desired_pose.orientation.y = -0.0855322959381
    desired_pose.orientation.z =  0.0364367915829
    desired_pose.orientation.w = -0.00293869199231
    interactive_marker.pose = desired_pose

    interactive_marker_init.markers.append(InteractiveMarker())

    positive_y_thresh = 0.20
    negative_y_thresh = -0.10
    rospy.loginfo("Publishing!!")

    while desired_pose.position.y < positive_y_thresh:

        desired_pose.position.y += 0.001
        rospy.loginfo("Pose(%f, %f, %f || %f, %f, %f, %f)", desired_pose.position.x, desired_pose.position.y, desired_pose.position.z, desired_pose.orientation.x, desired_pose.orientation.y, desired_pose.orientation.z, desired_pose.orientation.w)
        interactive_marker_init.seq_num = seq_num
        interactive_marker.pose = desired_pose
        interactive_marker_init.markers[0] = interactive_marker
        interactive_marker_init_publisher.publish(interactive_marker_init)
        seq_num = seq_num + 1
        rate.sleep()
    while desired_pose.position.y > negative_y_thresh:
        desired_pose.position.y -= 0.001
        rospy.loginfo("Pose(%f, %f, %f || %f, %f, %f, %f)", desired_pose.position.x, desired_pose.position.y, desired_pose.position.z, desired_pose.orientation.x, desired_pose.orientation.y, desired_pose.orientation.z, desired_pose.orientation.w)
        interactive_marker_init.seq_num = seq_num
        interactive_marker.pose = desired_pose
        interactive_marker_init.markers[0] = interactive_marker
        interactive_marker_init_publisher.publish(interactive_marker_init)
        seq_num = seq_num + 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
