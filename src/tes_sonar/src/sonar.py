#!/usr/bin/env python

from brping import Ping1D
import rospy
from std_msgs.msg import Int32
import time
import argparse

# Parse Command line options
############################
parser = argparse.ArgumentParser(description="Ping python library example with ROS.")
parser.add_argument('--device', action="store", required=False, type=str, help="Ping device port. E.g: /dev/ttyUSB0")
parser.add_argument('--baudrate', action="store", type=int, default=115200, help="Ping device baudrate. E.g: 115200")
parser.add_argument('--topic', action="store", required=False, type=str, default="/ping", help="ROS Topic to publish to. E.g: /ping")
args = parser.parse_args()

if args.device is None:
    parser.print_help()
    exit(1)

# Initialize ROS node
rospy.init_node('ping_sonar_node', anonymous=True)

# Create publisher to publish distance data
pub = rospy.Publisher(args.topic, Int32, queue_size=10)

# Make a new Ping object
myPing = Ping1D()
if args.device is not None:
    myPing.connect_serial(args.device, args.baudrate)

if myPing.initialize() is False:
    print("Failed to initialize Ping!")
    exit(1)

print("------------------------------------")
print("Starting Ping..")
print("Press CTRL+C to exit")
print("------------------------------------")

# Keep publishing the distance data to the ROS topic
while not rospy.is_shutdown():
    data = myPing.get_distance()
    if data:
        # Create a Int32 message with distance and confidence
        if data["confidence"] > 90: 
            print("data valid")
            message = data["distance"]
            rospy.loginfo(message)  # Log the data for debugging
            pub.publish(message)  # Publish to the ROS topic
    else:
        rospy.logwarn("Failed to get distance data")
    time.sleep(0.1)
