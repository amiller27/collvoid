import rospy
from collvoid_srvs.srv import GetObstacles
from visualization_msgs.msg import Marker

if __name__ == '__main__':
    rospy.init_node('obstacle_publisher')
    rospy.wait_for_service('/get_obstacles')
    get_obstacles = rospy.ServiceProxy('/get_obstacles', GetObstacles)
    pub = rospy.Publisher('/obstacles', Marker, queue_size=1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = get_obstacles()
        obstacles = msg.obstacles
        if not obstacles:
            rate.sleep()
            continue

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.type = 5
        marker.header.frame_id = obstacles[0].header.frame_id
        marker.ns = 'base_footprint'
        marker.id = 1
        marker.pose.orientation.w = 1
        marker.scale.x = 0.015
        marker.scale.y = 0
        marker.scale.z = 0
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1

        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = False
        for obstacle in obstacles:
            for i in range(4):
                marker.points.append(obstacle.polygon.points[i-1])
                marker.points.append(obstacle.polygon.points[i])
        pub.publish(marker)
        rate.sleep()
