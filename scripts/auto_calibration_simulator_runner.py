#! /usr/bin/python2

import argparse
import yaml
from pluspy import bag_utils, file_utils
import munch
import os
import subprocess
import tempfile
import shutil

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
			
			auto_calibration_config = b.get("auto_calibration_config")
			if not auto_calibration_config:
				auto_calibration_config = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../data/auto_calibration.prototxt")
			
			data_list.append(munch.Munch({
                            'problem_file': os.path.join(data_parent_dir,bag_filename+'/stereo_auto_calibration_problem.txt'),
							'bag_file': bag_filename,
							'bag_date': bag_components.dt_str,
                            'car': car,
							'common_config': common_config,
                            'stereo_config': stereo_config,
                            'lane_detection_config': lane_detection_config,
                            'auto_calibration_config': auto_calibration_config,
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
	parser.add_argument("-c", "--calib_dir", default='/opt/plusai/var/calib_db',
                        help="directory of calibration files",
                        metavar="CALIB_DIR", dest='calib_dir')
	parser.add_argument("-o", "--output_dir", default=os.path.dirname(os.path.realpath(__file__)),
                        help="output directory",
                        metavar="OUTPUT_DIR", dest='output_dir')
	args = parser.parse_args()
	data_list = parse_yaml_file(args.data_list, args.work_space)
	success_count = 0
	too_few_ground_count = 0
	two_view_fail_count = 0
	register_fail_count = 0
	cam_to_imu_fail_count = 0
	unknown_fail_count = 0
	file_utils.safe_make_dir(args.output_dir)
	f_result = open(os.path.join(args.output_dir,'run_rest.tsv'), 'w')
	for data in data_list:
		tmp_dir = tempfile.mkdtemp()
		out = None
		try:
			cmd = ['{WORKSPACE}/build/{BUILD}/bin/auto_calibration_simulator'.format(WORKSPACE=args.work_space, BUILD=BUILD),
				'--problem', data.problem_file,
				'--problem_parent_dir',os.path.dirname(data.problem_file),
				'--bag_file', data.bag_file,
				'--log_dir', "./",
				'--car', data.car,
				'--calib_dir', args.calib_dir,
				'--calib_date', data.bag_date,
				'--common_config', data.common_config,
				'--stereo_config', data.stereo_config,
				'--lane_config', data.lane_detection_config,
				'--calibration_config', data.auto_calibration_config,
				'--use_calib_date_in_common_config=False',
				'--alsologtostderr',
				'-v=5']
			out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, cwd=tmp_dir)
			# the following will print out the stderr 
			#out = subprocess.check_output(cmd,stderr=None, cwd=tmp_dir)
		except subprocess.CalledProcessError as e:
			#print("Process return ",e.returncode)
			#print("cmd", " ".join(e.cmd))
			if "failed to register snapshot" in e.output:
				register_fail_count = register_fail_count + 1
				f_result.write("{}\t failed to register snapshot\n".format(data.bag_file))
			elif "too few points in ground patch to look for ground plane" in e.output:
				too_few_ground_count = too_few_ground_count + 1
				f_result.write("{}\t too few points in ground patch to look for ground plane\n".format(data.bag_file))
			elif "failed to add first snapshot" in e.output:
				two_view_fail_count = two_view_fail_count + 1
				f_result.write("{}\t failed to add first snapshot\n".format(data.bag_file))
			elif "fail to create new cam to imu transform" in e.output:
				cam_to_imu_fail_count = cam_to_imu_fail_count +1
				f_result.write("{}\t fail to create new cam to imu transform\n".format(data.bag_file))
			else:
				unknown_fail_count = unknown_fail_count + 1
				f_result.write("{}\t unknown failure\n".format(data.bag_file))
			continue
		
		success_count = success_count + 1
		f_result.write("{}\t success\n".format(data.bag_file))

		for roots, dirs, files in os.walk(tmp_dir):

			for file in files:
				if ".yml" == os.path.splitext(file)[1]:
					old_name = os.path.join(roots,file)
					new_name = os.path.join(args.output_dir,file)
					shutil.move(old_name, new_name)
		
		old_name = data.problem_file + ".tsv"
		new_name = os.path.join(args.output_dir, "{}_{}".format(os.path.basename(data.bag_file),'stereoCalibratorReport.tsv'))
		shutil.move(old_name, new_name)
	f_result.close()
	print("Successfully ran {} / {} auto_calibration".format(success_count, len(data_list)))
	print("too_few_ground_count: {}, two_view_fail_count: {}, register_fail_count: {}, cam_to_imu_fail_count: {}, unknown_fail_count: {}".format(too_few_ground_count,two_view_fail_count,register_fail_count,cam_to_imu_fail_count, unknown_fail_count))

if __name__ == '__main__':
	main()
