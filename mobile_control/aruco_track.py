# #!/usr/bin/env python3
# import rospy
# import tf
# import math
# import numpy as np
# import tf.transformations as tft
# from geometry_msgs.msg import Twist
# from tf.transformations import euler_from_quaternion

# def cubic_trajectory(t, T, p0, pf):
#     a0 = p0
#     a1 = 0
#     a2 = 3 * (pf - p0) / (T ** 2)
#     a3 = -2 * (pf - p0) / (T ** 3)
#     return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3

# def get_tf_transform(listener, target_frame, source_frame):
#     try:
#         listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(3.0))
#         trans, rot = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
#         return trans, rot
#     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#         rospy.logwarn("TF lookup failed: %s -> %s", source_frame, target_frame)
#         return None, None

# def compute_relative_error(T_goal, T_current):
#     T_error = tft.concatenate_matrices(T_goal, tft.inverse_matrix(T_current))
#     dx = T_error[0, 3]
#     dy = T_error[1, 3]
#     R_error = T_error[0:3, 0:3]
#     _, _, dyaw = tft.euler_from_matrix(R_error)
#     dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))  # Normalize
#     return dx, dy, dyaw

# def main():
#     rospy.init_node('cubic_tf_trajectory_closed_loop')

#     # --- Frame Definitions ---
#     initial_frame = "aruco_frame"
#     camera_frame = "D435i_camera_front_color_optical_frame"
#     desired_camera_frame = "aruco_frame_desired"
#     base_frame = "mobile_base"

#     # --- Desired Goal Pose (in base_frame) ---
#     x_goal, y_goal, z_goal = 1.0, 0.5, 0.0
#     roll_goal, pitch_goal, yaw_goal = 0.0, 0.0, 0.8
#     T_base_goal = tft.concatenate_matrices(
#         tft.translation_matrix([x_goal, y_goal, z_goal]),
#         tft.euler_matrix(roll_goal, pitch_goal, yaw_goal)
#     )

#     tf_listener = tf.TransformListener()
#     cmd_pub = rospy.Publisher('/omnisteering/cmd_vel', Twist, queue_size=10)
#     rate = rospy.Rate(100)

#     rospy.loginfo("Waiting for initial pose from TF...")

#     # --- Wait for Initial Detection ---
#     while not rospy.is_shutdown():
#         init_trans, _ = get_tf_transform(tf_listener, camera_frame, initial_frame)
#         if init_trans:
#             break
#         rate.sleep()

#     # --- Closed-loop Control ---
#     while not rospy.is_shutdown():
#         trans, rot = get_tf_transform(tf_listener, "base_link", base_frame)
#         print(trans, rot)
#         trans, rot = get_tf_transform(tf_listener, base_frame, desired_camera_frame)
#         if trans is None:
#             rate.sleep()
#             continue

#         T_base_current = tft.concatenate_matrices(
#             tft.translation_matrix(trans),
#             tft.quaternion_matrix(rot)
#         )

#         # Debug: show updated transform each loop
#         rospy.loginfo("Current transform (T_base_current):\n%s", np.array_str(T_base_current, precision=3, suppress_small=True))

#         dx, dy, dyaw = compute_relative_error(T_base_goal, T_base_current)
#         error_norm = math.sqrt(dx**2 + dy**2 + (dyaw**2))

#         # Stop condition
#         if error_norm < 0.01:
#             rospy.loginfo("Target reached within tolerance. Stopping.")
#             break

#         # Trajectory duration
#         linear_error = math.hypot(dx, dy)
#         angular_error = abs(dyaw)
#         max_v = 0.2
#         max_w = 0.3
#         T = max(linear_error / max_v, angular_error / max_w)
#         T = max(1.0, min(T, 10.0))

#         # Execute trajectory
#         t0 = rospy.Time.now().to_sec()
#         while not rospy.is_shutdown():
#             t = rospy.Time.now().to_sec() - t0
#             if t > T:
#                 break

#             vx = cubic_trajectory(t, T, dx, 0)
#             vy = cubic_trajectory(t, T, dy, 0)
#             wz = cubic_trajectory(t, T, dyaw, 0)

#             vx = np.clip(vx, -0.3, 0.3)
#             vy = np.clip(vy, -0.3, 0.3)
#             wz = np.clip(wz, -0.4, 0.4)

#             if abs(vx) < 1e-3: vx = 0.0
#             if abs(vy) < 1e-3: vy = 0.0
#             if abs(wz) < 1e-3: wz = 0.0

#             cmd = Twist()
#             cmd.linear.x = vx
#             cmd.linear.y = vy
#             cmd.angular.z = wz
#             cmd_pub.publish(cmd)
#             rate.sleep()

#     cmd_pub.publish(Twist())
#     rospy.loginfo("Motion complete.")

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass


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

    source_frame = 'aruco_frame_first'
    target_frame = 'aruco_frame_second'

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
