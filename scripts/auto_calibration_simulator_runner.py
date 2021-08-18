#! /usr/bin/python2

import argparse
import yaml
from pluspy import file_utils
import os
import subprocess
import tempfile
import shutil

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
    parser.add_argument("-c", "--calib_dir", default='/opt/plusai/var/calib_db',
                        help="directory of calibration files",
                        metavar="CALIB_DIR", dest='calib_dir')
    parser.add_argument("-o", "--output_dir", default=os.path.dirname(os.path.realpath(__file__)),
                        help="output directory",
                        metavar="OUTPUT_DIR", dest='output_dir')
    args = parser.parse_args()
    data_list = utils.parse_yaml_file(args.data_list, args.work_space, BUILD)
    success_count = 0
    too_few_ground_count = 0
    two_view_fail_count = 0
    register_fail_count = 0
    cam_to_imu_fail_count = 0
    unknown_fail_count = 0
    file_utils.safe_make_dir(args.output_dir)
    f_result = open(os.path.join(args.output_dir, 'run_result.tsv'), 'w')
    for data in data_list:
        tmp_dir = tempfile.mkdtemp()
        out = None
        problem_file = data.bag_file + "/stereo_auto_calibration_problem.txt"
        try:
            cmd = ['{WORKSPACE}/build/{BUILD}/bin/auto_calibration_simulator'.format(WORKSPACE=args.work_space, BUILD=BUILD),
                '--problem', problem_file,
                '--problem_parent_dir', data.bag_file,
                '--bag_file', data.bag_file,
                '--log_dir', "./",
                '--logger_default_log_destinations', "./",
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
            #out = subprocess.check_output(cmd, stderr=None, cwd=tmp_dir)
        except subprocess.CalledProcessError as e:
            #print("Process return ",e.returncode)
            #print("cmd", " ".join(e.cmd))
            with open(os.path.join(args.output_dir, data.bag_basename + ".txt"), 'w') as fout:
                fout.write(e.output)
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
                cam_to_imu_fail_count = cam_to_imu_fail_count + 1
                f_result.write("{}\t fail to create new cam to imu transform\n".format(data.bag_file))
            else:
                unknown_fail_count = unknown_fail_count + 1
                f_result.write("{}\t unknown failure\n".format(data.bag_file))
            continue

        success_count = success_count + 1
        f_result.write("{}\t success\n".format(data.bag_file))

        for roots, dirs, files in os.walk(tmp_dir):

            for file in files:
                #print(file)
                if ".yml" == os.path.splitext(file)[1]:
                    old_name = os.path.join(roots, file)
                    new_name = os.path.join(args.output_dir, file)
                    shutil.move(old_name, new_name)

        old_name = problem_file + ".tsv"
        new_name = os.path.join(args.output_dir, "{}_{}".format(os.path.basename(data.bag_file), 'stereoCalibratorReport.tsv'))
        shutil.move(old_name, new_name)
    f_result.close()
    print("Successfully ran {} / {} auto_calibration".format(success_count, len(data_list)))
    print("too_few_ground_count: {}, two_view_fail_count: {}, register_fail_count: {}, cam_to_imu_fail_count: {}, unknown_fail_count: {}".format(too_few_ground_count, two_view_fail_count, register_fail_count, cam_to_imu_fail_count, unknown_fail_count))


if __name__ == '__main__':
    main()
