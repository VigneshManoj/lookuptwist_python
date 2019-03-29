# !/usr/bin/env python


import rospy
from numpy.linalg import inv
import tf
import geometry_msgs.msg
import math
import numpy as np
import quaternion
from sympy import symbols, Eq, solve
from pyquaternion import Quaternion as Quat
from PyKDL import Rotation, Vector, Frame
from scipy.spatial.transform import Rotation as scipy_rot


# -------------
# def vel_calc_func(start, end, corrected_average_interval):
#
#
#     temp = inv(start[:3, :3]) * end[:3, :3]
#     print "temp mat is ", temp
#     quat = Quaternion(matrix=temp)
#     o = start[:3, :3] * quat.axis
#     ang = quat.angle
#     delta_x = end[:3, 0:1] - start[:3, 0:1]
#     delta_y = end[:3, 1:2] - start[:3, 1:2]
#     delta_z = end[:3, 2:3] - start[:3, 2:3]
#
#     twist_vel = geometry_msgs.msg.Vector3()
#     twist_vel.x = delta_x / rospy.Duration.to_sec(corrected_average_interval)
#     twist_vel.y = delta_y / rospy.Duration.to_sec(corrected_average_interval)
#     twist_vel.z = delta_z / rospy.Duration.to_sec(corrected_average_interval)
#     # twist_rot = geometry_msgs.msg.Vector3()
#     twist_rot = o* (ang/rospy.Duration.to_sec(corrected_average_interval))
#
#     return twist_vel, twist_rot

# -------------

def vel_calc_func(start_mat, end_mat, corrected_average_interval):

    # start_r = scipy_rot.from_quat([start_quat[0], start_quat[1], start_quat[2], start_quat[3]])
    # start_r = quaternion.as_rotation_matrix(start_quat)
    # quat_start = Quat(start_quat)

    # quat_end = Quat(end_quat)
    start_r = start_mat.M
    end_r = end_mat.M
    # print "start rot ", start_r
    # print "end rot ", end_r
    # print " inv start rot ", start_mat.Inverse()
    temp = start_mat.M.Inverse() * end_mat.M
    temp_mat = Frame(Rotation(temp))
    # print "temp ", temp
    # print "temp mat is ", temp_mat
    # quat = Rotation.GetQuaternion(temp)

    # np_quat = Quat(quat)
    # print "quat value is ", np_quat.axis
    ang, temp_axis = Rotation.GetRotAngle(temp)
    o = start_mat.M * temp_axis

    # print " o ", o
    # mat_start = Frame(Rotation.Quaternion(start_quat[0], start_quat[1], start_quat[2], start_quat[3]), Vector(start_pos[0], start_pos[1], start_pos[2]))

    p_start_vec = start_mat.p
    # mat_end = Frame(Rotation.Quaternion(end_quat[0], end_quat[1], end_quat[2], end_quat[3]), Vector(end_pos[0], end_pos[1], end_pos[2]))
    p_end_vec = end_mat.p
    # print "p vectors ", p_end_vec, p_start_vec
    delta_x = p_end_vec[0] - p_start_vec[0]
    print "delta x is ", delta_x
    delta_y = p_end_vec[1] - p_start_vec[1]
    delta_z = p_end_vec[2] - p_start_vec[2]

    twist_vel = geometry_msgs.msg.Vector3()
    twist_vel.x = delta_x / corrected_average_interval.to_sec()
    twist_vel.y = delta_y / corrected_average_interval.to_sec()
    twist_vel.z = delta_z / corrected_average_interval.to_sec()
    # twist_rot = geometry_msgs.msg.Vector3()
    twist_rot = o* (ang/corrected_average_interval.to_sec())
    print "twist vel is ", twist_vel
    print "twist rot is ", twist_rot[2]
    return twist_vel, twist_rot



# def get_rotation_matrix(self):
#     """Return the rotation matrix which this quaternion is equivalent to
#
#     Returns:
#         The rotation matrix which this quaternion is equivalent to as a
#         list of three lists of three elements each
#     """
#     rot_matx = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
#     rot_matx[0][0] = (math.pow(self.w, 2) + math.pow(self.x, 2) -
#                       math.pow(self.y, 2) - math.pow(self.z, 2))
#     rot_matx[0][1] = (2*self.x*self.y - 2*self.w*self.z)
#     rot_matx[0][2] = (2*self.x*self.z + 2*self.w*self.y)
#     rot_matx[1][0] = (2*self.x*self.y + 2*self.w*self.z)
#     rot_matx[1][1] = (math.pow(self.w, 2) - math.pow(self.x, 2) +
#                       math.pow(self.y, 2) - math.pow(self.z, 2))
#     rot_matx[1][2] = (2*self.y*self.z - 2*self.w*self.x)
#     rot_matx[2][0] = (2*self.x*self.z - 2*self.w*self.y)
#     rot_matx[2][1] = (2*self.y*self.z + 2*self.w*self.x)
#     rot_matx[2][2] = (math.pow(self.w, 2) - math.pow(self.x, 2) -
#                       math.pow(self.y, 2) + math.pow(self.z, 2))
#     return rot_matx

def get_rotation_matrix(mat):

    rot_matx = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    rot_matx[0][0] = (math.pow(mat[3], 2) + math.pow(mat[0], 2) -
                      math.pow(mat[1], 2) - math.pow(mat[2], 2))
    rot_matx[0][1] = (2*mat[0]*mat[1] - 2*mat[3]*mat[2])
    rot_matx[0][2] = (2*mat[0]*mat[2] + 2*mat[3]*mat[1])
    rot_matx[1][0] = (2*mat[0]*mat[1] + 2*mat[3]*mat[2])
    rot_matx[1][1] = (math.pow(mat[3], 2) - math.pow(mat[0], 2) + math.pow(mat[1], 2) - math.pow(mat[2], 2))
    rot_matx[1][2] = (2*mat[1]*mat[2] - 2*mat[3]*mat[0])
    rot_matx[2][0] = (2*mat[0]*mat[2] - 2*mat[3]*mat[1])
    rot_matx[2][1] = (2*mat[1]*mat[2] + 2*mat[3]*mat[0])
    rot_matx[2][2] = (math.pow(mat[3], 2) - math.pow(mat[0], 2) -   math.pow(mat[1], 2) + math.pow(mat[2], 2))
    return rot_matx

def rotationMatrixToEulerAngles(R):

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        theta_x = math.atan2(R[2, 1], R[2, 2])
        theta_y = math.atan2(-R[2, 0], sy)
        theta_z = math.atan2(R[1, 0], R[0, 0])
    else:
        theta_x = math.atan2(-R[1, 2], R[1, 1])
        theta_y = math.atan2(-R[2, 0], sy)
        theta_z = 0

    return np.array([theta_x, theta_y, theta_z])


if __name__ == '__main__':
    rospy.init_node('davinci_tf_listener')

    listener = tf.TransformListener()
    # print "Reached here"
    xyz_pos_val_pub = rospy.Publisher('xyz_pos_val_pub', geometry_msgs.msg.WrenchStamped, queue_size=1)
    # accel_val_pub = rospy.Publisher('accel_val_pub', geometry_msgs.msg.Vector3, queue_size=1)
    xyz_pos_val_pub2 = rospy.Publisher('xyz_pos_val_pub2', geometry_msgs.msg.WrenchStamped, queue_size=1)

    position_pub = rospy.Publisher('final_position_pub', geometry_msgs.msg.Vector3, queue_size=1)
    avg_val_pub = rospy.Publisher('avg_pub', geometry_msgs.msg.Vector3, queue_size=1)

    rate = rospy.Rate(500.0)
    while not rospy.is_shutdown():
        try:


            # Taking the position, linear and angular velocity of insertion link wrt psm1_base_link
            # (trans_ins, rot_ins) = listener.lookupTransform('psm1_base_link', 'psm1_main_insertion_link', rospy.Time(0))
            # (lin_twist_ins, ang_twist_ins) = listener.lookupTwist('psm1_base_link', 'psm1_main_insertion_link',
            #                                                       rospy.Time(0), rospy.Duration(0.1))
            #
            # # Taking the position, linear and angular velocity of psm1_yaw_link link wrt base_link
            (trans_psm1_base, rot_psm1_base) = listener.lookupTransform('base_link', 'psm1_base_link', rospy.Time(0))
            # (lin_twist_psm1_yaw, ang_twist_psm1_yaw) = listener.lookupTwist('base_link', 'psm1_yaw_link', rospy.Time(0),
            #                                                       rospy.Duration(0.1))
            tracking_frame = 'base_link'
            reference_frame = 'psm1_main_insertion_link'
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
            start_pos, start_quat = listener.lookupTransform(reference_frame, tracking_frame, start_time)
            # print "start quat is ", start_quat
            end_pos, end_quat = listener.lookupTransform(reference_frame, tracking_frame, end_time)
            start_mat = Frame(Rotation.Quaternion(start_quat[0], start_quat[1], start_quat[2], start_quat[3]), Vector(start_pos[0], start_pos[1], start_pos[2]))
            end_mat = Frame(Rotation.Quaternion(end_quat[0], end_quat[1], end_quat[2], end_quat[3]),
                            Vector(end_pos[0], end_pos[1], end_pos[2]))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print "----------------------starts here------------------------------------"

        twist_vel, twist_rot = vel_calc_func(start_mat, end_mat, corrected_average_interval)
        # print twist_vel, twist_rot
        xyz_pub = geometry_msgs.msg.WrenchStamped()
        xyz_pub.header.frame_id = 'psm1_main_insertion_link'
        # rotmat_ins_org = listener.fromTranslationRotation(trans_psm1_base, rot_psm1_base)
        # print "twist vel is" , twist_vel
        # force_val2 = inv(rotmat_ins_org[:3, :3]) * np.array([[twist_vel.x], [twist_vel.y], [twist_vel.z]])
        # print "force val ", force_val2[0][0], force_val2[1][0], force_val2[2][0]
        # xyz_pub.wrench.force.x = force_val2[0][0]
        # xyz_pub.wrench.force.y = force_val2[1][0]
        # xyz_pub.wrench.force.z = force_val2[2][0]
        xyz_pub.wrench.force.x = twist_vel.x
        xyz_pub.wrench.force.y = twist_vel.y
        xyz_pub.wrench.force.z = twist_vel.z
        xyz_pos_val_pub.publish(xyz_pub)

        rate.sleep()