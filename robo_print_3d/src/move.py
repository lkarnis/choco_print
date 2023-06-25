#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
from std_msgs.msg import Float32
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        d = dist((x1, y1, z1), (x0, y0, z0))
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class Move(object):
    """Move"""

    def __init__(self):
        super(Move, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_choco", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()

        eef_link = move_group.get_end_effector_link()

        group_names = robot.get_group_names()

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        #publisher za motor
        self.motor_speed_publisher = rospy.Publisher('motor_speed', Float32, queue_size=10)
        self.temp_set1_publisher = rospy.Publisher('temp_set1', Float32, queue_size=10)
        self.temp_set2_publisher = rospy.Publisher('temp_set2', Float32, queue_size=10)

    def plan_cartesian_path(self, waypoints):
        move_group = self.move_group
        move_group.set_max_velocity_scaling_factor(0.001)
        move_group.set_max_acceleration_scaling_factor(0.01)
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.0008, 0.0
        )
        return plan, fraction


    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)
    

def main():
    try:
        choco = Move()
        
        rospy.sleep(1)
        choco.motor_speed_publisher.publish(Float32(0.0))
        choco.temp_set1_publisher.publish(Float32(36.0))
        choco.temp_set2_publisher.publish(Float32(34.0))

        # Collecting three points from the user for plane calculation
        points = []
        for i in range(3):
            input("Postavi robota u točku {} i pritisni Enter".format(i+1))
            current_pose = choco.move_group.get_current_pose().pose
            points.append([current_pose.position.x, current_pose.position.y, current_pose.position.z])
        points = np.array(points)

        # Calculating the plane normal
        v1 = points[1] - points[0]
        v2 = points[2] - points[0]
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)
        
        # Ensure that the normal is pointing upwards
        if normal[2] < 0:
            normal = -normal

        # Converting the normal into a quaternion orientation
        axis = np.cross([0, 0, 1], normal)
        axis = axis / np.linalg.norm(axis)
        angle = np.arccos(np.dot([0, 0, 1], normal))
        quat = R.from_rotvec(axis * angle).as_quat()
        quat = quat.tolist()

        # Define a transformation matrix from the origin to the first point on the plane
        T1 = np.eye(4)
        T1[:3, 3] = points[0]

        # Define a rotation matrix that aligns the z-axis with the normal
        R_z = R.from_rotvec(axis * angle).as_matrix()

        # Create the final transformation matrix
        T = np.eye(4)
        #T[:3, :3] = R_z
        T = np.matmul(T1, T)
        
        # Spremanje trenutne orijentacije alata prije početka kretanja
        initial_orientation = choco.move_group.get_current_pose().pose.orientation

        # The robot will move in the plane defined by the user's points
        waypoints = []
        
        filename = "filtered_coordinates.txt"
        with open(filename, 'r') as file:
            lines = file.readlines()



        for line in lines:
            coordinates = line.strip().split(' ')
            pose = geometry_msgs.msg.Pose()
            pose.position.x = float(coordinates[0])
            pose.position.y = float(coordinates[1])
            pose.position.z = float(coordinates[2])
            pose.orientation = initial_orientation  # Postavljanje fiksne orijentacije alata
            waypoints.append(copy.deepcopy(pose))
            
        pose = geometry_msgs.msg.Pose()
        pose.position.z += 0.1
        pose.position.y += 0.1
        pose.orientation = initial_orientation  # Postavljanje fiksne orijentacije alata
        waypoints.append(copy.deepcopy(pose))


        # Transform the coordinates to the new coordinate system
        for waypoint in waypoints:
            old_coords = np.array([waypoint.position.x, waypoint.position.y, waypoint.position.z, 1])
            new_coords = np.matmul(T, old_coords)
            waypoint.position.x = new_coords[0]
            waypoint.position.y = new_coords[1]
            waypoint.position.z = new_coords[2]
            
        cartesian_plan, fraction = choco.plan_cartesian_path(waypoints)
        choco.display_trajectory(cartesian_plan)

        # Splitaj na 20
        waypoint_chunks = [waypoints[i:i + 100] for i in range(0, len(waypoints), 100)]

        input("Pritisni Enter za početak printanja")
        choco.motor_speed_publisher.publish(Float32(10.0))

        # šalji po 20
        for chunk in waypoint_chunks:
            cartesian_plan, fraction = choco.plan_cartesian_path(chunk)
            choco.execute_plan(cartesian_plan)

        choco.motor_speed_publisher.publish(Float32(0.0))
        print("Printanje završeno")
    except rospy.ROSInterruptException:
        choco.motor_speed_publisher.publish(Float32(0.0))
        return
    except KeyboardInterrupt:
        choco.motor_speed_publisher.publish(Float32(0.0))
        return
    finally:
        choco.motor_speed_publisher.publish(Float32(0.0))




if __name__ == "__main__":
    main()

