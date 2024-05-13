#!/usr/bin/env python3
import rospy
import os
from time import sleep
#sudo chown root:root alarm_node.py 
#sudo chmod u+s alarm_node.py
import RPi.GPIO as GPIO
from time import sleep
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
import math
from dynamic_reconfigure.server import Server
from alarm.cfg import ObstacleDetectionConfig

# Define global variables
# Offsets
X_OFFSET = 0.5  # X offset in meters
Y_OFFSET = 0.2  # Y offset in meters
ALARM_ZONE_FRONT_LAT = 0.3  # Lateral ±30cm
ALARM_ZONE_FRONT_LONG = 0.4  # Longitudinal 40cm
ALARM_ZONE_FRONT_LONG_INIT = 0.4
ALARM_POINTCLOUD_TOPIC = '/alarm_pointcloud'  # Topic name for the alarm point cloud
ALARM_BOUNDARY_TOPIC = '/alarm_boundary'  # Topic name for the boundary of the alarm area

# Initialize Publishers
pointcloud_pub = rospy.Publisher(ALARM_POINTCLOUD_TOPIC, PointCloud, queue_size=10)
boundary_pub = rospy.Publisher(ALARM_BOUNDARY_TOPIC, PointCloud, queue_size=10)

# GPIO BCM
GPIO.setmode(GPIO.BCM)

# GPIO outputmode
led_pin = 17
led_pin1 = 27
GPIO.setup(led_pin, GPIO.OUT)
GPIO.setup(led_pin1, GPIO.OUT)

def publish_alarm_boundary():
    """Publish the boundary points of the alarm area"""
    boundary = PointCloud()
    boundary.header.stamp = rospy.Time.now()
    boundary.header.frame_id = "laser"  # Adjust based on actual frame_id

    # Calculate boundary points to form a complete rectangle
    num_points_per_side = 20  # Number of points per side
    num_points_per_side_lr = 80
    points = []

    # Left and right sides with offsets
    for i in range(num_points_per_side_lr + 1):
        x = i * (4 * ALARM_ZONE_FRONT_LONG - ALARM_ZONE_FRONT_LONG_INIT) / num_points_per_side_lr + ALARM_ZONE_FRONT_LONG_INIT + X_OFFSET
        points.append(Point32(x, -ALARM_ZONE_FRONT_LAT + Y_OFFSET, 0))
        points.append(Point32(x, ALARM_ZONE_FRONT_LAT + Y_OFFSET, 0))

    # Top and bottom sides with offsets
    for i in range(1, num_points_per_side):
        y = i * 2 * ALARM_ZONE_FRONT_LAT / num_points_per_side - ALARM_ZONE_FRONT_LAT + Y_OFFSET
        points.append(Point32(ALARM_ZONE_FRONT_LONG_INIT + X_OFFSET, y, 0))
        points.append(Point32(4 * ALARM_ZONE_FRONT_LONG + X_OFFSET, y, 0))

    boundary.points = points
    boundary_pub.publish(boundary) 
    #rospy.loginfo("Alarm boundary published.")

def distance(x, y):
    """Calculate the distance from the origin to the point (x, y)"""
    return math.sqrt(x * x + y * y)

def reconfigure_callback(config, level):
    """Callback function for dynamic reconfigure requests"""
    global ALARM_ZONE_FRONT_LAT, ALARM_ZONE_FRONT_LONG, X_OFFSET, Y_OFFSET
    ALARM_ZONE_FRONT_LAT = config.alarm_zone_front_lat
    ALARM_ZONE_FRONT_LONG = config.alarm_zone_front_long
    X_OFFSET = config.x_offset
    Y_OFFSET = config.y_offset
    rospy.loginfo("Updated configuration to: Lateral distance = {:.2f} m, Longitudinal distance = {:.2f} m".format(ALARM_ZONE_FRONT_LAT, ALARM_ZONE_FRONT_LONG))
    return config

def process_scan(msg):
    # print(ALARM_ZONE_FRONT_LONG, ALARM_ZONE_FRONT_LAT)
    """Process each LaserScan message"""
    publish_alarm_boundary()
    cloud = PointCloud()
    cloud.header.stamp = rospy.Time.now()
    cloud.header.frame_id = msg.header.frame_id  # Use the same frame_id

    # Flag to determine if an alarm should be triggered
    alarm_triggered = False

    # Iterate through all laser points
    for i, range in enumerate(msg.ranges):
        # Calculate the current point's angle
        angle = msg.angle_min + i * msg.angle_increment

        # Calculate the Cartesian coordinates (x forward, y left)
        x = range * math.cos(angle)
        y = range * math.sin(angle)

        # Ignore invalid range readings
        if range == float('inf') or range == float('-inf') or math.isnan(range):
            continue

        # Filter out points outside lateral ±30cm
        if abs(y - Y_OFFSET) > ALARM_ZONE_FRONT_LAT:
            continue

        # Determine if the point is within the non-alarming longitudinal area
        if x < ALARM_ZONE_FRONT_LONG_INIT + X_OFFSET or x > 4 * ALARM_ZONE_FRONT_LONG + X_OFFSET:
            continue
        
        if distance(x, y) < 0.1:
            continue

        # If an obstacle is detected within the alarm area, add to the point cloud and trigger the alarm
        alarm_triggered = True
        cloud.points.append(Point32(x, y, 0))  # z is set to 0 because it's a 2D point cloud

    # Publish the alarm point cloud
    if alarm_triggered:
        pointcloud_pub.publish(cloud)
        rospy.loginfo("Alarm! Obstacle detected and pointcloud published.")
        GPIO.output(led_pin, GPIO.HIGH)
        GPIO.output(led_pin1, GPIO.HIGH)

        #os.system('sudo python3 on.py')
    else:
        rospy.loginfo("No Obstacle. All clear.")
        GPIO.output(led_pin, GPIO.LOW)
        GPIO.output(led_pin1, GPIO.LOW)

        #os.system('sudo python3 off.py')
        
def hook():
    GPIO.output(led_pin, GPIO.LOW)
    GPIO.output(led_pin1, GPIO.LOW)

    GPIO.cleanup()
    
def alarm_node():
    """Main function to initialize the node"""
    rospy.init_node('obstacle_alarm_node')
    srv = Server(ObstacleDetectionConfig, reconfigure_callback)
    
    publish_alarm_boundary()
    
    rospy.Subscriber("/scan", LaserScan, process_scan)

    rospy.on_shutdown(hook)

    rospy.spin()

if __name__ == '__main__':
    try:
        alarm_node()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        GPIO.cleanup()
        pass
    finally:
        GPIO.cleanup()
