#! /usr/bin/python2

import argparse
import yaml
from pluspy import bag_utils, file_utils
import munch
import os
import subprocess
import tempfile
import shutil
from load_bag import load_rosbag_from_file
import utils
BUILD = "relwithdebinfo"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--datalist", required=True,
                    help="yaml file containing bags to process",
                    metavar="FILE", dest='data_list')
    parser.add_argument("-w", "--workspace", required=True,
                    help="drive repo, example: /work/drive",
                    metavar="WORKSPACE", dest='work_space')
    parser.add_argument("-c", "--calib_dir", default='/opt/plusai/var/calib_db/',
                    help="directory of calibration files",
                    metavar="CALIB_DIR", dest='calib_dir')
    parser.add_argument("-o", "--output_dir", default=os.path.dirname(os.path.realpath(__file__)),
                    help="output directory",
                    metavar="OUTPUT_DIR", dest='output_dir')
    args = parser.parse_args()
    data_list = utils.parse_yaml_file(args.data_list, args.work_space, BUILD)
    file_utils.safe_make_dir(args.output_dir)
    for data in data_list:
        temp_dir = tempfile.mkdtemp()
        print(temp_dir)
        bag = load_rosbag_from_file(data.bag_file)
        topics = " ".join(bag.get_type_and_topic_info().topics.keys())
        # print(topics)
        bag.close()
        import re

        def search_pattern(patterns, topics):
            for pattern in patterns:
                topic = re.search(pattern, topics)
                if topic is not None:
                    return topic.group(0)
            return None

        front_left_topic = search_pattern(["/front_left_camera/image_(color|raw)/compressed"], topics)
        front_right_topic = search_pattern(["/front_right_camera/image_(color|raw)/compressed"], topics)
        rear_left_topic = search_pattern(["/rear_left_camera/image_(color|raw)/compressed"], topics)
        rear_right_topic = search_pattern(["/rear_right_camera/image_(color|raw)/compressed"], topics)
        lidar_topic = search_pattern(['/unified/[a-z]*_points', '/unified/points', '/unified_egofilter/[a-z]*_points',
'/os1_left/[a-z]*points'], topics)

        if front_left_topic is None:
            front_left_topic = "NoTopic"
        if front_right_topic is None:
            front_right_topic = "NoTopic"
        rear_cameras_present = 'true'
        if rear_left_topic is None:
            rear_left_topic = "NoTopic"
            rear_cameras_present = 'false'
        if rear_right_topic is None:
            rear_right_topic = 'NoTopic'
            rear_cameras_present = 'false'
        lidar_present = 'true'
        if lidar_topic is None:
            lidar_topic = 'NoTopic'
            lidar_present = 'false'
        out = None
        try:
            env = {'GLOG_v': '5', 'PLUSAI_LOGGER_LOG_DIRECTORY': temp_dir}
            cmd = ['{WORKSPACE}/build/{BUILD}/bin/unified_perception_simulator'.format(WORKSPACE=args.work_space, BUILD=BUILD),
            '--bag_file', data.bag_file,
            '--out_image_path', temp_dir,
            '--car', data.car,
            '--show_active_sensors=false',
            '--show_fusion_tracker_monitor=false',
            '--show_last_update_info=false',
            '--show_tracks=true',
            '--show_new_tracks=false',
            '--show_track_id=true',
            '--show_track_vel=true',
            '--show_track_pos=true',
            '--show_detection_boxes=true',
            '--show_detection_prob=true',
            '--show_camera_detection_dists=true',
            '--show_detection_orientation=true',
            '--show_cuboid=false',
            '--show_obstacle_pointcloud=true',
            '--show_lanes=true',
            '--show_roi_pcl_on_image=true',
            '--show_occupancy_grid=true',
            '--show_tracks_ortho=true',
            '--show_velocity_vector=true',
            '--interactive=false',
            '--calib_dir', args.calib_dir,
            '--front_left_topic', front_left_topic,
            '--front_right_topic', front_right_topic,
            '--rear_left_topic', rear_left_topic,
            '--rear_right_topic', rear_right_topic,
            '--lidar_topic', lidar_topic,
            '--common_config', data.common_config,
            '--stereo_config', data.stereo_config,
            '--fusion_config', data.fusion_config,
            '--lane_detection_config', data.lane_detection_config,
            '--map_param_file', data.map_param_file,
            '--side_cameras_present=true',
            '--rear_cameras_present={}'.format(rear_cameras_present),
            '--lidar_present={}'.format(lidar_present),
            '--log_timer_vlog_level=100',
            '--use_calib_date_in_common_config=False',
            '--logger_log_verbosity_level', '5',
            '--logger_enable_log_levels=all',
            '--logger_enable_log_levels=all,warning,error,catastrophe,assert_failure,info',
            '--logger_log_directory', temp_dir,
            '--out_dataset_path', "dataset"]
            #print(" ".join(cmd))
            #exit(0)
            out = subprocess.check_output(cmd, env=env)
        except subprocess.CalledProcessError as e:
            print(" ".join(e.cmd))

        for roots, dirs, files in os.walk(temp_dir):
            for file in files:
                result = search_pattern(['^unified_perception_simulator\.[0-9\.\-_]+\.log$'], file)
                if result is not None:
                    # move file to temp dir
					old_name = os.path.join(roots,file)
					# print(old_name)
					new_name = os.path.join(args.output_dir, data.bag_basename + "_" + result)
					shutil.move(old_name, new_name)
		#shutil.rmtree(temp_dir, ignore_errors=True)

      

if __name__ == '__main__':
    main()
