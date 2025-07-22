#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def load_velocities_from_txt(file_path):
    velocities = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("#") or len(line) == 0:
                continue
            parts = line.split(',')
            if len(parts) < 3:
                continue
            vx, vy, wz = map(float, parts[:3])
            velocities.append((vx, vy, wz))
    return velocities

def main():
    rospy.init_node('open_loop_velocity_player')
    cmd_pub = rospy.Publisher('/omnisteering/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(100)  # 0.01 seconds per command

    # === Load velocities ===
    velocity_file = "/home/mlei/concert_ws/cubic_trajectory_position_mobile.txt"
    rospy.loginfo("Reading velocity file: %s", velocity_file)
    velocity_list = load_velocities_from_txt(velocity_file)
    rospy.loginfo("Loaded %d velocity commands", len(velocity_list))

    rospy.sleep(1.0)  # optional wait before start
    
    i = 0
    for vx, vy, wz in velocity_list:
        if rospy.is_shutdown():
            break
        cmd = Twist()
        cmd.linear.x = vx 
        cmd.linear.y = vy 
        cmd.angular.z = wz * 0
        i = i +1
        print(i)
        cmd_pub.publish(cmd)
        rate.sleep()

    # Send zero velocity at end
    cmd_pub.publish(Twist())
    rospy.loginfo("Finished replaying velocity commands.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
