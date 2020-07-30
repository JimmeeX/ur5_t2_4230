# MTRN4230 Group 6 Assignment
# Checkpoint 2
# Simple robot motion


from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy

waypoints = [[0.3445, -1.265, 1.496, 1.339, 1.571, 1.920],
             [4.586, -1.020, 1.7, 0.889, 1.571, -0.126],
             [3.3441, -0.2379, -2.324, 0.991, -1.5708, -1.369]]

def send_movement_command():
    rospy.init_node('send_joints')
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    traj = JointTrajectory()
    traj.header = Header()
    joint.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                         'wrist_joint_1', 'wrist_joint_2', 'wrist_joint_3']
    rate = rospy.Rate(0.5)
    count = 0
    pts = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        count += 1
        pts.positions = waypoints[count%len(waypoints)]
        pts.time_from_start = rospy.Duration(1.0)

        traj.points = []
        traj.points.append(pts)
        pub.publihs(traj)
        rate.sleep()


if __name__ == '__main__':
    try:
        send_movement_command()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
