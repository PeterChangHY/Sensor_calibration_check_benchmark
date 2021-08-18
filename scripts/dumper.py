"""A script to dump the sync raw data from bag/fastbag
    now it will dump 
    front_left_camera, 
    front_right_camera,
    front_radar,
    odometry,

"""

#!/usr/bin/env python

import argparse
import euler
import cv2
from cv_bridge import CvBridge
import os
import tf
import numpy as np
from pluspy import file_utils
import math
from load_bag import (
    load_rosbag_from_file,
    buffered_message_generator,
)
import tempfile
import shutil
import json

import utils
import re
import pypcd

bridge = CvBridge()


def imgmsg_to_cv2(imgmsg):
    try:
        if 'compressed' in imgmsg.topic:
            res = bridge.compressed_imgmsg_to_cv2(imgmsg.message, desired_encoding="passthrough")
        else:
            res = bridge.imgmsg_to_cv2(imgmsg.message, desired_encoding="passthrough")
    except AttributeError as e:
        return None
    return res


def writeOdomMsg(odom_msg, file_path):
    """WriteOdomMsg in to file

    Args:
        odom_msg:
            Odometry message
        file_path:
            A file_path to dump the data. if the file existed, then it will overwrite.

    Returns:
        None

    """
    odom = odom_msg.message
    bag_time = odom_msg.timestamp.to_sec()
    msg_time = odom.header.stamp.to_sec()
    orientation_matrix = tf.transformations.quaternion_matrix(
        [
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ])
    roll, pitch, yaw = euler.recover_euler(orientation_matrix)[0]
    with open(file_path, "w") as o:
        out_dict = {
            "msg_time": msg_time,
                "bag_time": bag_time,
                "x": odom.pose.pose.position.x,
                "y": odom.pose.pose.position.y,
                "z": odom.pose.pose.position.z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "qx": odom.pose.pose.orientation.x,
                "qy": odom.pose.pose.orientation.y,
                "qz": odom.pose.pose.orientation.z,
                "qw": odom.pose.pose.orientation.w,
                "vx": odom.twist.twist.linear.x,
                "vy": odom.twist.twist.linear.y,
                "vz": odom.twist.twist.linear.z
        }
        o.write(json.dumps(out_dict))

def writeRadarMsg(radar_msg, file_path):
    radar_message = radar_msg.message
    radar_list = []
    pointcloud_data_list= []
    for i in range(len(radar_message.tracks)):
        radar_track = {
            "timestamp": radar_message.header.stamp.to_sec(),
            "track_id": radar_message.tracks[i].track_id,
            "velocity": [
                    radar_message.tracks[i].linear_velocity.x,
                    radar_message.tracks[i].linear_velocity.y,
                    radar_message.tracks[i].linear_velocity.z,
            ],
            "acceleration": [
                    radar_message.tracks[i].linear_acceleration.x,
                    radar_message.tracks[i].linear_acceleration.y,
                    radar_message.tracks[i].linear_acceleration.z,
            ],
            "track_shape": [(p.x, p.y, p.z) for p in radar_message.tracks[i].track_shape.points],
        }
        radar_list.append(radar_track)

    with open(file_path, "w") as o:
        o.write(json.dumps(radar_list))


def writeRadarMsgPCD(radar_msg, file_path):
    radar_message = radar_msg.message
    radar_list = []
    pointcloud_data_list= []
    for i in range(len(radar_message.tracks)):
        for p in radar_message.tracks[i].track_shape.points:
            pointcloud_data_list.append([p.x, p.y, p.z])

    # convet it to pcd
    pointcloud_data = np.array(pointcloud_data_list)
    pypcd.pypcd.save_point_cloud_bin(
        pypcd.make_xyz_point_cloud(pointcloud_data), file_path
    )

def writeLidarMsg(lidar_msg, file_path):
    pc = pypcd.PointCloud.from_msg(lidar_msg.message)
    pc.save(file_path)

def getTopics(extraced_topics):
    """get the necessary topics

    Args:
        extraced_topics:
            all topics existed in the bag
    Return:
        topics:
            a dict of the necessary topics for data dumping if all necessary topics can be found,
            otherwise will reutrn None

    """

    def search_pattern(patterns, topics):
        for topic in topics:
            for pattern in patterns:
                topic = re.search(pattern, topic)
                if topic is not None:
                    return topic.group(0)
        return None

    topics = {}
    front_left_topic = search_pattern(["/front_left_camera/image_(color|raw)/compressed"], extraced_topics)
    if front_left_topic is None:
        print("Can't find front_left_camera topic")
        return None
    topics['front_left_camera'] = front_left_topic
    
    front_right_topic = search_pattern(["/front_right_camera/image_(color|raw)/compressed"], extraced_topics)
    if front_right_topic is None:
        print("Can't find front_right_camera topic")
        return None
    topics['front_right_camera'] = front_right_topic
    
    odom_topic = search_pattern(["/navsat/odom"], extraced_topics)
    if front_right_topic is None:
        print("Can't find odom topic")
        return None
    topics['odom'] = odom_topic
    
    front_radar_topic = search_pattern(["(^/conti_bumper_radar/conti_bumper_radar/radar_tracks$|^/conti_bumper_radar/radar_tracks$|^/bumper_radar/radar_tracks$|^/front_center_radar/radar_tracks$|^/front_bumper_radar/radar_tracks$)"], extraced_topics)
    if front_radar_topic is None:
        print("Can't find front_radar topic")
        return None
    topics['front_radar'] = front_radar_topic

    left_corner_radar_topic = search_pattern(["^/left_corner_radar/radar_tracks"], extraced_topics)
    if left_corner_radar_topic is not None:
        topics['left_corner_radar'] = left_corner_radar_topic
    else :
        print("Can't find left_corner_radar_topic")
    right_corner_radar_topic = search_pattern(["^/right_corner_radar/radar_tracks"], extraced_topics)
    if right_corner_radar_topic is not None:
        topics['right_corner_radar'] = right_corner_radar_topic
    else :
        print("Can't find right_corner_radar_topic")
    
    left_lidar_topic = search_pattern(["^/os1_left/points"], extraced_topics)
    if left_lidar_topic is not None:
        topics['left_lidar'] = left_lidar_topic
    else :
        print("Can't find left_lidar_topic")
    right_lidar_topic = search_pattern(["^/os1_right/points"], extraced_topics)
    if right_lidar_topic is not None:
        topics['right_lidar'] = right_lidar_topic
    else :
        print("Can't find right_lidar_topic")
    return topics

if __name__ == "__main__":
    parser = argparse.ArgumentParser("datadumper")
    parser.add_argument("--bags", type=str, nargs='*',
                        help="bag_paths")
    parser.add_argument("-d", "--datalist",
                        help="yaml file containing bags to process, if exist it will add into bags",
                        metavar="FILE", dest='data_list')
    parser.add_argument(
        "--rate",
        type=float,
        default=0.2,
        help="how often to dump a message, in seconds",
    )

    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.03,
        help="max gap to allow between message pairs",
    )

    parser.add_argument(
        "-n",
        type=int,
        default=10,
        help="max number of frames",
        dest="max_number_of_frames"
    )
    parser.add_argument("--output_dir", type=str, default="./", help="output directory")
    args = parser.parse_args()

    outdir = os.path.expanduser(args.output_dir)
    outdir = os.path.abspath(outdir)

    bags = args.bags
    if bags is None:
        bags = []

    # add bags in yaml files
    if args.data_list:
        data_list = utils.parse_yaml_file(args.data_list, "foo_ws", "foo_build")
        for data in data_list:
            bags.append(data.bag_file)

    for bag in bags:
        bag_instance = load_rosbag_from_file(bag)
        bag_all_topics = getTopics(bag_instance.get_type_and_topic_info().topics.keys())
        if bag_all_topics is None:
            print(bag)
            print(bag_instance.get_type_and_topic_info().topics.keys())
            print('fail')
            continue

        front_left_topic = bag_all_topics['front_left_camera']
        front_right_topic = bag_all_topics['front_right_camera']
        front_radar_topic = bag_all_topics['front_radar']
        odom_topic = bag_all_topics['odom']


        extraced_topics = [front_left_topic, front_right_topic, front_radar_topic, odom_topic]

        # optional topic will extrac if exist
        left_lidar_topic = "foo"
        if "left_lidar" in bag_all_topics:
            left_lidar_topic = bag_all_topics["left_lidar"]
            extraced_topics.append(left_lidar_topic)
        
        right_lidar_topic = "foo"
        if "right_lidar" in bag_all_topics:
            right_lidar_topic = bag_all_topics["right_lidar"]
            extraced_topics.append(right_lidar_topic)
        
        left_corner_radar_topic = "foo"
        if "left_corner_radar" in bag_all_topics:
            left_corner_radar_topic = bag_all_topics["left_corner_radar"]
            extraced_topics.append(left_corner_radar_topic)
        
        right_corner_radar_topic = "foo"
        if "right_corner_radar" in bag_all_topics:
            right_corner_radar_topic = bag_all_topics["right_corner_radar"]
            extraced_topics.append(right_corner_radar_topic)
        
        out_subdir = os.path.join(outdir, os.path.basename(bag))

        # prepare output directories for each sensor
        sub_dirs = {}
        # print(extraced_topics)
        for topic in extraced_topics:
            # print(topic)
            sub_dirs[topic] =  os.path.join(out_subdir, topic.split("/")[1])

        count = 0

        # temporary dict
        temp_file_dict = {}
        stereo_problem_filename = os.path.join(out_subdir, 'stereo_auto_calibration_problem.txt')
        temp_file_dict[stereo_problem_filename] = tempfile.mkstemp(suffix='.txt')[1]
        #print("file", temp_file_dict[stereo_problem_filename])
        stereo_problem_f = open(temp_file_dict[stereo_problem_filename], 'w')

        tmp_dir = tempfile.mkdtemp()



        def writeImg2Temp(img_msg, temp_file_dict, count, dir):
            img = imgmsg_to_cv2(img_msg)
            if img is None:
                return None
            img_filename = os.path.join(dir, "{:04d}.jpg".format(count))
            temp_file_dict[img_filename] = tempfile.mkstemp(suffix='.jpg')[1]
            cv2.imwrite(temp_file_dict[img_filename], img)
            return True
        
        def writeOdom2Temp(odom_msg, temp_file_dict, count, dir):
            odom_filename = os.path.join(dir, "{:04d}.json".format(count))
            temp_file_dict[odom_filename] = tempfile.mkstemp(suffix='.json')[1]
            writeOdomMsg(odom_msg, temp_file_dict[odom_filename])
            return True
        
        def writeRadar2Temp(radar_msg, temp_file_dict, count, dir):
            radar_filename = os.path.join(dir, "{:04d}.json".format(count))
            temp_file_dict[radar_filename] = tempfile.mkstemp(suffix='.json')[1]
            writeRadarMsg(radar_msg, temp_file_dict[radar_filename])
            writeRadarPCD2Temp(radar_msg, temp_file_dict, count, dir)
            return True
        
        def writeRadarPCD2Temp(radar_msg, temp_file_dict, count, dir):
            radar_filename = os.path.join(dir, "{:04d}.pcd".format(count))
            temp_file_dict[radar_filename] = tempfile.mkstemp(suffix='.pcd')[1]
            writeRadarMsgPCD(radar_msg, temp_file_dict[radar_filename])
            return True
        
        def writeLidar2Temp(lidar_msg, temp_file_dict, count, dir):
            lidar_filename = os.path.join(dir, "{:04d}.pcd".format(count))
            temp_file_dict[lidar_filename] = tempfile.mkstemp(suffix='.pcd')[1]
            writeLidarMsg(lidar_msg,temp_file_dict[lidar_filename])
            return True



        for frame in buffered_message_generator(bags=[bag_instance], topics=extraced_topics, tolerance=args.tolerance, rate=args.rate):

            if front_left_topic not in frame:
                continue
            front_left_img_message = frame[front_left_topic]
            if writeImg2Temp(front_left_img_message, temp_file_dict, count, sub_dirs[front_left_topic]) is None:
                continue

            if front_right_topic not in frame:
                continue
            front_right_img_message = frame[front_right_topic]
            if writeImg2Temp(front_right_img_message, temp_file_dict, count, sub_dirs[front_right_topic]) is None:
                continue

            if front_radar_topic not in frame:
                continue
            front_radar_message = frame[front_radar_topic]
            if writeRadar2Temp(front_radar_message, temp_file_dict, count, sub_dirs[front_radar_topic]) is None:
                continue
            
            if left_lidar_topic in frame:
                if not writeLidar2Temp( frame[left_lidar_topic], temp_file_dict, count, sub_dirs[left_lidar_topic]):
                    print("failed to write left_lidar")
            if right_lidar_topic in frame:
                if not writeLidar2Temp( frame[right_lidar_topic], temp_file_dict, count, sub_dirs[right_lidar_topic]):
                    print("failed to write right_lidar")
            
            if left_corner_radar_topic in frame:
                writeRadar2Temp(  frame[left_corner_radar_topic], temp_file_dict, count, sub_dirs[left_corner_radar_topic])

            if right_corner_radar_topic in frame:
                writeRadar2Temp(frame[right_corner_radar_topic], temp_file_dict, count, sub_dirs[right_corner_radar_topic])



            if odom_topic not in frame:
                continue
            odom_message = frame[odom_topic]
            if writeOdom2Temp(odom_message, temp_file_dict, count, sub_dirs[odom_topic]) is None:
                continue

            # write stereo_problem
            front_left_relpath = os.path.relpath(os.path.join(sub_dirs[front_left_topic], "{:04d}.jpg".format(count)), out_subdir)
            front_right_relpath = os.path.relpath(os.path.join(sub_dirs[front_right_topic], "{:04d}.jpg".format(count)), out_subdir)
            problem_text = " ".join([front_left_relpath, front_right_relpath])
            stereo_problem_f.write(problem_text + '\n')
            count = count + 1
            if count >= args.max_number_of_frames:
                break

        stereo_problem_f.close()

        if count >= args.max_number_of_frames:
            file_utils.safe_make_dir(out_subdir)
            for topic in extraced_topics:
                file_utils.safe_make_dir(sub_dirs[topic])
            for key in temp_file_dict:
                shutil.move(temp_file_dict[key], key)

            print("Success to dump data from {}, count = {}".format(bag, count))
        else:
            print("Fail to dump data from {}, count = {}".format(bag, count))
