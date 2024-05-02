#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
import math
from dynamic_reconfigure.server import Server
from alarm.cfg import ObstacleDetectionConfig

# 定义全局变量
ALARM_ZONE_FRONT_LAT = 0.3  # 横向±30cm
ALARM_ZONE_FRONT_LONG = 0.4  # 纵向40cm
ALARM_POINTCLOUD_TOPIC = '/alarm_pointcloud'  # 报警点云的话题名称
ALARM_BOUNDARY_TOPIC = '/alarm_boundary'  # 报警区域边界的话题名称

# 初始化Publisher
pointcloud_pub = rospy.Publisher(ALARM_POINTCLOUD_TOPIC, PointCloud, queue_size=10)
boundary_pub = rospy.Publisher(ALARM_BOUNDARY_TOPIC, PointCloud, queue_size=10)

def publish_alarm_boundary():
    """发布警报区域的边界点"""
    boundary = PointCloud()
    boundary.header.stamp = rospy.Time.now()
    boundary.header.frame_id = "laser"  # 根据实际的frame_id调整

    # 计算边界点，形成完整的矩形框
    num_points_per_side = 20  # 每边的点数
    points = []
    # 上下边
    for i in range(num_points_per_side + 1):
        x = i * 4 * ALARM_ZONE_FRONT_LONG / num_points_per_side
        points.append(Point32(x, -ALARM_ZONE_FRONT_LAT, 0))
        points.append(Point32(x, ALARM_ZONE_FRONT_LAT, 0))
    # 左右边
    for i in range(1, num_points_per_side):
        y = i * 2 * ALARM_ZONE_FRONT_LAT / num_points_per_side - ALARM_ZONE_FRONT_LAT
        points.append(Point32(0, y, 0))
        points.append(Point32(4 * ALARM_ZONE_FRONT_LONG, y, 0))

    boundary.points = points
    boundary_pub.publish(boundary)
    rospy.loginfo("Alarm boundary published.")
    
def distance (x, y):
    return math.sqrt(x * x + y * y)

def reconfigure_callback(config, level):
    global ALARM_ZONE_FRONT_LAT, ALARM_ZONE_FRONT_LONG
    ALARM_ZONE_FRONT_LAT = config.alarm_zone_front_lat
    ALARM_ZONE_FRONT_LONG = config.alarm_zone_front_long
    rospy.loginfo("Updated configuration to: Lateral distance = {:.2f} m, Longitudinal distance = {:.2f} m".format(ALARM_ZONE_FRONT_LAT, ALARM_ZONE_FRONT_LONG))
    return config
    
def process_scan(msg):
    # 创建PointCloud消息
    cloud = PointCloud()
    cloud.header.stamp = rospy.Time.now()
    cloud.header.frame_id = msg.header.frame_id  # 使用相同的frame_id

    # 用于确定是否触发警报的标志
    alarm_triggered = False

    # 遍历所有的激光点
    for i, range in enumerate(msg.ranges):
        # 计算当前点的角度
        angle = msg.angle_min + i * msg.angle_increment

        # 计算该点的笛卡尔坐标（x向前，y向左）
        x = range * math.cos(angle)
        y = range * math.sin(angle)
        
        # 忽略无效的范围读数
        if range == float('inf') or range == float('-inf') or math.isnan(range):
            continue

        # 滤除横向±30cm以外的点
        if abs(y) > ALARM_ZONE_FRONT_LAT:
            continue
            
        # 滤除纵向±30cm以外的点
        # if abs(y) > ALARM_ZONE_FRONT_LAT:
        #     continue

        # 判断是否在不报警的纵向区域内
        if x < ALARM_ZONE_FRONT_LONG:
            continue
            
        # 判断是否在不报警的纵向区域内
        if x > 4 * ALARM_ZONE_FRONT_LONG:
            continue
        
        if distance(x, y) < 0.1 :
            continue

        # 如果检测到障碍物在报警区域内，则加入点云并触发警报
        alarm_triggered = True
        cloud.points.append(Point32(x, y, 0))  # z设置为0，因为是2D点云

    # 发布报警点云
    if alarm_triggered:
        pointcloud_pub.publish(cloud)
        rospy.loginfo("Alarm! Obstacle detected and pointcloud published.")
    else:
        rospy.loginfo("No Obstacle. All clear.")

def alarm_node():
    rospy.init_node('obstacle_alarm_node')
    srv = Server(ObstacleDetectionConfig, reconfigure_callback)
    
    publish_alarm_boundary()
    
    # 创建Subscriber，订阅LaserScan消息
    rospy.Subscriber("/scan", LaserScan, process_scan)

    # 保持程序继续运行直到节点被关闭
    rospy.spin()

if __name__ == '__main__':
    try:
        alarm_node()
    except rospy.ROSInterruptException:
        pass

