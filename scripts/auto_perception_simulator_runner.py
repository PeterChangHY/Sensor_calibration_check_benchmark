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

BUILD = "relwithdebinfo"


def parse_yaml_file(filename, workspace):
    data_list = []
    with open(filename, 'rb') as fh:
        raw_data = yaml.safe_load(fh)
        # handle `~` in Unix
        data_parent_dir = os.path.expanduser(raw_data.get('parent_dir', [])[0])
        for b in raw_data.get("bag_dirs", []):
            bag_filename = b.get("bag")
            bag_components = bag_utils.extract_bag_components(bag_filename)

            car = b.get("car")
            if not car:
                if bag_components is None:
                    print("Can't extract_bag_components for bag: {}".format(bag_filename))
                    exit(1)
                car = bag_components.vehicle

            common_config = b.get("common_config")
            if not common_config:
                common_config = "{WORKSPACE}/opt/{BUILD}/config/common_config.prototxt.{VEHICLE_NAME}".format(
                    WORKSPACE=workspace,
                    BUILD=BUILD,
                    VEHICLE_NAME=car
                )
            stereo_config = b.get("stereo_config")
            if not stereo_config:
                stereo_config = "{WORKSPACE}/opt/{BUILD}/config/stereo_tracker.prototxt.{VEHICLE_NAME}".format(
                    WORKSPACE=workspace,
                    BUILD=BUILD,
                    VEHICLE_NAME=car
                )
            lane_detection_config = b.get("lane_detection_config")
            if not lane_detection_config:
                lane_detection_config = "{WORKSPACE}/opt/{BUILD}/config/lane_detection_lane_marking.prototxt.{VEHICLE_NAME}".format(
                    WORKSPACE=workspace,
                    BUILD=BUILD,
                    VEHICLE_NAME=car
                )
            fusion_config = b.get("fusion_config")
            if not fusion_config:
                fusion_config = "{WORKSPACE}/opt/{BUILD}/config/fusion_tracker.prototxt.{VEHICLE_NAME}".format(
                    WORKSPACE=workspace,
                    BUILD=BUILD,
                    VEHICLE_NAME=car
                )
            map_param_file = b.get("map_param_file")
            if not map_param_file:
                map_param_file = "{WORKSPACE}/opt/{BUILD}/config//map_param.prototxt.{VEHICLE_NAME}".format(
                    WORKSPACE=workspace,
                    BUILD=BUILD,
                    VEHICLE_NAME=car
                )

            auto_calibration_config = b.get("auto_calibration_config")
            if not auto_calibration_config:
                auto_calibration_config = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../data/auto_calibration.prototxt")

            data_list.append(munch.Munch({
                'bag_file': os.path.join(data_parent_dir, bag_filename),
                'bag_date': bag_components.dt_str,
                'bag_basename': os.path.basename(bag_filename),
                'car': car,
                'common_config': common_config,
                'stereo_config': stereo_config,
                'lane_detection_config': lane_detection_config,
                'auto_calibration_config': auto_calibration_config,
                                'fusion_config': fusion_config,
                                'map_param_file': map_param_file,
            }))

    return data_list


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
    data_list = parse_yaml_file(args.data_list, args.work_space)
    file_utils.safe_make_dir(args.output_dir)
    for data in data_list:
        temp_dir = tempfile.mkdtemp()
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
            '--logger_log_directory', temp_dir]
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
					#print(old_name)
					new_name = os.path.join(args.output_dir, data.bag_basename + "_" + result)
					shutil.move(old_name, new_name)
		#shutil.rmtree(temp_dir, ignore_errors=True)

      

if __name__ == '__main__':
    main()
