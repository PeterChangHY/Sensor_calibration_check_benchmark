"""this script includes the utility functions

"""
import yaml
import munch
from pluspy import bag_utils
import os

def parse_yaml_file(filename, workspace, build):
    data_list = []
    if not os.path.isfile(filename):
        return data_list
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
                    BUILD=build,
                    VEHICLE_NAME=car
                )
            stereo_config = b.get("stereo_config")
            if not stereo_config:
                stereo_config = "{WORKSPACE}/opt/{BUILD}/config/stereo_tracker.prototxt.{VEHICLE_NAME}".format(
                    WORKSPACE=workspace,
                    BUILD=build,
                    VEHICLE_NAME=car
                )
            lane_detection_config = b.get("lane_detection_config")
            if not lane_detection_config:
                lane_detection_config = "{WORKSPACE}/opt/{BUILD}/config/lane_detection_lane_marking.prototxt.{VEHICLE_NAME}".format(
                    WORKSPACE=workspace,
                    BUILD=build,
                    VEHICLE_NAME=car
                )
            fusion_config = b.get("fusion_config")
            if not fusion_config:
                fusion_config = "{WORKSPACE}/opt/{BUILD}/config/fusion_tracker.prototxt.{VEHICLE_NAME}".format(
                    WORKSPACE=workspace,
                    BUILD=build,
                    VEHICLE_NAME=car
                )
            map_param_file = b.get("map_param_file")
            if not map_param_file:
                map_param_file = "{WORKSPACE}/opt/{BUILD}/config//map_param.prototxt.{VEHICLE_NAME}".format(
                    WORKSPACE=workspace,
                    BUILD=build,
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
