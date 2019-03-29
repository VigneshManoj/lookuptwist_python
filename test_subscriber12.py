# !/usr/bin/env python


import rospy
import tf
import geometry_msgs.msg
import math
import numpy as np
from PyKDL import Rotation, Vector, Frame




def vel_calc_func(start_mat, end_mat, corrected_average_interval):


    # Create rotation matrices
    start_r = start_mat.M
    end_r = end_mat.M

    # Transform matrices using inverse for correct orientation
    temp = start_mat.M.Inverse() * end_mat.M
    temp_mat = Frame(Rotation(temp))

    ang, temp_axis = Rotation.GetRotAngle(temp)
    o = start_mat.M * temp_axis
    p_start_vec = start_mat.p
    p_end_vec = end_mat.p
    # print "p vectors ", p_end_vec, p_start_vec
    delta_x = p_end_vec[0] - p_start_vec[0]
    print "delta x is ", delta_x
    delta_y = p_end_vec[1] - p_start_vec[1]
    delta_z = p_end_vec[2] - p_start_vec[2]

    # Assign values to a Vector3 element
    twist_vel = geometry_msgs.msg.Vector3()
    twist_vel.x = delta_x / corrected_average_interval.to_sec()
    twist_vel.y = delta_y / corrected_average_interval.to_sec()
    twist_vel.z = delta_z / corrected_average_interval.to_sec()
    # twist_rot = geometry_msgs.msg.Vector3()
    twist_rot = o* (ang/corrected_average_interval.to_sec())
    print "twist vel is ", twist_vel
    print "twist rot is ", twist_rot[2]
    return twist_vel, twist_rot

# Main function
if __name__ == '__main__':
    rospy.init_node('davinci_tf_listener')

    listener = tf.TransformListener()
    # print "Reached here"
    # Declaring a ros publisher
    xyz_pos_val_pub = rospy.Publisher('xyz_pos_val_pub', geometry_msgs.msg.WrenchStamped, queue_size=1)

    rate = rospy.Rate(500.0)
    while not rospy.is_shutdown():
        try:

            # Taking the position, linear and angular velocity of psm1_base_link link wrt base_link
            (trans_psm1_base, rot_psm1_base) = listener.lookupTransform('base_link', 'psm1_base_link', rospy.Time(0))
            # (lin_twist_psm1_yaw, ang_twist_psm1_yaw) = listener.lookupTwist('base_link', 'psm1_yaw_link', rospy.Time(0),
            #                                                       rospy.Duration(0.1))

            # Declaring tracking and reference frames
            tracking_frame = 'base_link'
            reference_frame = 'psm1_main_insertion_link'

            #Calculation of time and averaging time interval
            time = rospy.Time(0)
            average_interval = rospy.Duration(0.1)
            # Taking the position, linear and angular velocity of psm1_pitch_back link wrt base_link
            latest_time = listener.getLatestCommonTime(reference_frame, tracking_frame)
            if rospy.Time() == time:
                target_time = latest_time
            else:
                target_time = time
            end_time = min(target_time + average_interval * 0.5, latest_time)
            start_time = max(rospy.Time.from_seconds(0.00001) + average_interval, end_time) - average_interval
            corrected_average_interval = end_time - start_time

            # Get translation and quaternion of tf frame
            start_pos, start_quat = listener.lookupTransform(reference_frame, tracking_frame, start_time)
            # print "start quat is ", start_quat

            # Get translation and quaternion of tf frame
            end_pos, end_quat = listener.lookupTransform(reference_frame, tracking_frame, end_time)

            # Create 4x4 homogeneous transformation matrix using position and quaternion values
            start_mat = Frame(Rotation.Quaternion(start_quat[0], start_quat[1], start_quat[2], start_quat[3]), Vector(start_pos[0], start_pos[1], start_pos[2]))
            end_mat = Frame(Rotation.Quaternion(end_quat[0], end_quat[1], end_quat[2], end_quat[3]),
                            Vector(end_pos[0], end_pos[1], end_pos[2]))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Call the velocity function
        twist_vel, twist_rot = vel_calc_func(start_mat, end_mat, corrected_average_interval)
        # print twist_vel, twist_rot

        # Publish the output as wrench topic so that it can be visualized
        xyz_pub = geometry_msgs.msg.WrenchStamped()
        xyz_pub.header.frame_id = 'psm1_main_insertion_link'
        xyz_pub.wrench.force.x = twist_vel.x
        xyz_pub.wrench.force.y = twist_vel.y
        xyz_pub.wrench.force.z = twist_vel.z
        xyz_pos_val_pub.publish(xyz_pub)

        rate.sleep()