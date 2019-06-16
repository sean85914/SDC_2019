#!/usr/bin/env python
import rospy
import rospkg
import csv
import sys

from argparse import ArgumentParser
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def readData(filename):
    data = []
    with open(filename) as csvfile:
        rows = csv.reader(csvfile)
        for row in rows:
            p = [row[1], row[2], row[3]]
            data.append(p)
    return data

def main():
    rospy.init_node("plot_gt")
    pub = rospy.Publisher("~ground_truth", Marker, queue_size = 10)
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('final_localization')
    bag_num = int(sys.argv[1])
    print "bag num: ", bag_num
    if bag_num == 1:
        GTfile = package_path + "/localization_ground_truth/localization_ground_truth_1.csv"
    elif bag_num ==2:
        GTfile = package_path + "/localization_ground_truth/localization_ground_truth_2.csv"
    else:
        print "No input file given, aborting..."
        sys.exit(0)
    data = readData(GTfile)
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.5
    marker.color.r = marker.color.b = marker.color.a = 1.0
    for i in range(len(data)):
        p = Point()
        p.x = float(data[i][0])
        p.y = float(data[i][1])
        p.z = float(data[i][2])
        marker.points.append(p)
        
    
    while not rospy.is_shutdown():
        pub.publish(marker)
    
    
if __name__ == "__main__":
    main()
