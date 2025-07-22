#!/usr/bin/env python
import rospy
import tf
import numpy as np
import tf.transformations as tft
import matplotlib.pyplot as plt


def cubic_trajectory(t, T, p0, pf):
    a0 = p0
    a1 = 0
    a2 = 3 * (pf - p0) / (T ** 2)
    a3 = -2 * (pf - p0) / (T ** 3)
    return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3


def cubic_trajectory_derivative(t, T, p0, pf):
    a1 = 0
    a2 = 6 * (pf - p0) / (T ** 2)
    a3 = -6 * (pf - p0) / (T ** 3)
    return a1 + a2 * t + a3 * t ** 2


def generate_cubic_path(t, T, start_pos, goal_pos):
    x_t = cubic_trajectory(t, T, start_pos[0], goal_pos[0])
    y_t = cubic_trajectory(t, T, start_pos[1], goal_pos[1])
    yaw_t = cubic_trajectory(t, T, start_pos[2], goal_pos[2])
    return [x_t, y_t, yaw_t]


def get_transform(listener, from_frame, to_frame):
    listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
    T = np.dot(tft.translation_matrix(trans), tft.quaternion_matrix(rot))
    return T, rot


def get_relative_transform(listener, source_frame, target_frame):
    world_frame = 'mobile_base'  # common ancestor frame
    T_world_to_source,_ = get_transform(listener, world_frame, source_frame)
    T_world_to_target,rot = get_transform(listener, world_frame, target_frame)
    print(T_world_to_target)
    print(rot)
    T_source_to_target = np.dot(T_world_to_target, tft.inverse_matrix(T_world_to_source))
    return T_source_to_target


def main():
    rospy.init_node('tf_relative_trajectory_plotter', anonymous=True)
    listener = tf.TransformListener()

    source_frame = 'aruco_marker_5'
    target_frame = 'second_desire'

    try:
        # Step 1: Get relative transform
        T_a_to_b = get_relative_transform(listener, source_frame, target_frame)
        rel_translation = T_a_to_b[:3, 3]
        rel_quaternion = tft.quaternion_from_matrix(T_a_to_b)
        rel_euler = tft.euler_from_matrix(T_a_to_b, axes='sxyz')

        goal_pos = [rel_translation[0], rel_translation[1], rel_euler[2]]  # x, y, yaw
        start_pos = [0.0, 0.0, 0.0]

        T_total = 50.0  # total time
        dt = 0.001       # time step

        # Storage
        trajectory_x, trajectory_y, trajectory_yaw = [], [], []
        velocity_x, velocity_y, velocity_yaw = [], [], []
        time_stamps = []

        print("\n--- Cubic Trajectory (x, y, yaw) ---")
        t = 0.0
        while t <= T_total:
            pos_t = generate_cubic_path(t, T_total, start_pos, goal_pos)
            vel_t = [
                cubic_trajectory_derivative(t, T_total, start_pos[0], goal_pos[0]),
                cubic_trajectory_derivative(t, T_total, start_pos[1], goal_pos[1]),
                cubic_trajectory_derivative(t, T_total, start_pos[2], goal_pos[2])
            ]

            # print("t = {:.1f}s -> Pos: [x, y, yaw] = {}, Vel: {}".format(t, pos_t, vel_t))

            trajectory_x.append(pos_t[0])
            trajectory_y.append(pos_t[1])
            trajectory_yaw.append(pos_t[2])

            velocity_x.append(vel_t[0])
            velocity_y.append(vel_t[1])
            velocity_yaw.append(vel_t[2])

            time_stamps.append(t)
            t += dt

        # Save position file
        with open("cubic_trajectory_position_mobile.txt", 'w') as f_pos:
            # f_pos.write("# t [s], x [m], y [m], yaw [rad]\n")
            for i in range(len(time_stamps)):
                f_pos.write("{:.6f}, {:.6f}, {:.6f}\n".format(
                     trajectory_x[i], trajectory_y[i], trajectory_yaw[i]
                ))

        # Save velocity file
        with open("cubic_trajectory_velocity_mobile.txt", 'w') as f_vel:
            # f_vel.write("# t [s], vx [m/s], vy [m/s], vyaw [rad/s]\n")
            for i in range(len(time_stamps)):
                f_vel.write(" {:.6f}, {:.6f}, {:.6f}\n".format(
                    velocity_x[i], velocity_y[i], velocity_yaw[i]
                ))

        print("\nSaved to:")
        print(" - cubic_trajectory_position_mobile.txt")
        print(" - cubic_trajectory_velocity_mobile.txt")

        # Plot trajectorycubic_trajectory_position_mobile.txt
        plt.figure(figsize=(8, 6))
        plt.plot(trajectory_x, trajectory_y, marker='o', label='Cubic Trajectory')
        plt.quiver(
            trajectory_x, trajectory_y,
            np.cos(trajectory_yaw), np.sin(trajectory_yaw),
            scale=10, width=0.005, color='r', label='Yaw direction'
        )
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('2D Cubic Trajectory: {} → {}'.format(source_frame, target_frame))
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF lookup failed: %s", str(e))


if __name__ == '__main__':
    main()
