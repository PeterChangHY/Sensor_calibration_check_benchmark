#! /usr/bin/python2

import argparse
import os
import pandas as pd
import json
import re


def parseUnifiedPerceptionSimulatorLog(file):
    with open(file) as f:
        dicts = []
        for line in f:
            # https://github.com/PlusAI/drive/blob/master/perception/obstacle_detection/src/fusion/fusion_tracker.cpp#L1682
            if 'New Detection:' in line:
                dict_string = line[line.find('New Detection:') + 15:-1]
                try:
                    dict = eval(dict_string, {'Infinity': -1})
                except NameError:
                    print(dict_string)
                    exit(1)
                    # pass

                dicts.append(dict)

    return pd.DataFrame.from_records(dicts)


def getStereoCalibrationfile(file):
    with open(file) as f:
        for line in f:
            if 'load StereoCaliration:' in line:
                return line[line.find('load StereoCaliration:') + 24:-2]


STEREO_FUSED_TYPE = ["RLS", "LS", "RS"]


def extractStereoFusedTracks(df):
    return df[df["fused_det_sensor"].apply(lambda x: x in STEREO_FUSED_TYPE)]


def extractRadarStereoFusedTracks(df):
    try:
        res = df[df["fused_det_sensor"] == 'RS']
    except KeyError:
        res = None
    return res

def maxTrack(df, key):
    if df.shape[0] == 0:
        return 0
    else:
        return df[key].max()


master_dir = os.path.expanduser('~/Downloads/20210709_auto_calibration_benchmark/original_calib_steteo_calibration_percepion_simulator')
openCV_dir = os.path.expanduser('~/Downloads/20210709_auto_calibration_benchmark/opencv_calib_steteo_calibration_percepion_simulator')
laneVP_dir = os.path.expanduser('~/Downloads/20210709_auto_calibration_benchmark/laneVanishingPoint_calib_steteo_calibration_percepion_simulator')
target_name = 'Batch_optimize'
target_dir = os.path.expanduser('~/Downloads/20210709_auto_calibration_benchmark/Batch_perception_simulator')

def get_all_files(dir):
    file_list = []
    for root, dirs, files in os.walk(dir):
        for file in files:
            file_list.append(os.path.join(root, file))
    return file_list

def keep_lastet_files(all_files):
    filtered_files = {}
    for file in all_files:
        key = re.search('.+\.db', os.path.basename(file)).group(0)
        if key in filtered_files:
            if file > filtered_files[key]:
                #print(file, "to repalce ",filtered_files[key])
                filtered_files[key] = file
                
        else:
            filtered_files[key] = file
    return_files = []
    for key in filtered_files:
        return_files.append(filtered_files[key])
    return return_files
            


def getBagName(filename):
    return re.search('.+\.db', os.path.basename(filename)).group(0)


master_files = get_all_files(master_dir)
master_files.sort(key=getBagName)
opencv_files = get_all_files(openCV_dir)
opencv_files.sort(key=getBagName)
laneVP_files = get_all_files(laneVP_dir)
laneVP_files.sort(key=getBagName)
target_files = keep_lastet_files(get_all_files(target_dir))
target_files.sort(key=getBagName)
print(len(master_files), len(opencv_files), len(laneVP_files), len(target_files))
#exit()

for file_master, file_openCV, file_laneVP, file_target in zip(master_files, opencv_files, laneVP_files, target_files):
    print(os.path.basename(file_master))
    bag_name_master = re.search('.+\.db', os.path.basename(file_master)).group(0)
    bag_name_opencv = re.search('.+\.db', os.path.basename(file_openCV)).group(0)
    bag_name_lanevp = re.search('.+\.db', os.path.basename(file_laneVP)).group(0)
    bag_name_target = re.search('.+\.db', os.path.basename(file_target)).group(0)

    if bag_name_master != bag_name_opencv:
        print(bag_name_master)
        print(bag_name_opencv)
        exit(1)
    if bag_name_master != bag_name_lanevp:
        print(bag_name_master)
        print(bag_name_lanevp)
        exit(1)
    if bag_name_master != bag_name_target:
        print(bag_name_master)
        print(bag_name_target)
        exit(1)
    master_detections = parseUnifiedPerceptionSimulatorLog(file_master)
    Opencv_detections = parseUnifiedPerceptionSimulatorLog(file_openCV)
    laneVP_detections = parseUnifiedPerceptionSimulatorLog(file_laneVP)
    target_detections = parseUnifiedPerceptionSimulatorLog(file_target)

    if master_detections.empty or Opencv_detections.empty or laneVP_detections.empty or target_detections.empty:
        continue
    #print('master calibration:', getStereoCalibrationfile(file_master))
    print('master number_of_radar_stereo_fused_detection:', extractRadarStereoFusedTracks(master_detections).shape[0])
    #print('master number_of_stereo_fused_detection:', extractStereoFusedTracks(master_detections).shape[0])
    print('master MAX xrel of_stereo_fused_detection:', maxTrack(extractStereoFusedTracks(master_detections), 'xrel'))
    Opencv_detections = parseUnifiedPerceptionSimulatorLog(file_openCV)
    #print('Opencv calibration:', getStereoCalibrationfile(file_openCV))
    print('Opencv_detections number_of_stereo_fused_detection:', extractStereoFusedTracks(Opencv_detections).shape[0])
    print('Opencv_detections_detections MAX xrel of_stereo_fused_detection:', maxTrack(extractStereoFusedTracks(Opencv_detections), 'xrel'))
    laneVP_detections = parseUnifiedPerceptionSimulatorLog(file_laneVP)
    #print('LaneVP calibration:', getStereoCalibrationfile(file_laneVP))
    print('LaneVP_detections number_of_stereo_fused_detection:', extractStereoFusedTracks(laneVP_detections).shape[0])
    print('LaneVP_detections MAX xrel of_stereo_fused_detection:', maxTrack(extractStereoFusedTracks(laneVP_detections), 'xrel'))

    print('{}_detections number_of_stereo_fused_detection:'.format(target_name), extractStereoFusedTracks(target_detections).shape[0])
    print('{}_detections MAX xrel of_stereo_fused_detection:'.format(target_name), maxTrack(extractStereoFusedTracks(target_detections), 'xrel'))

    import matplotlib as mpl
    mpl.use('Agg')
    import matplotlib.pyplot as plt

    def getXrelDiff(df):
        x = df['xrelRadar'].tolist()
        y = df['xrelStereo'].tolist()
        y = [u - v for u, v in zip(x, y)]
        return x, y

    master_radar_stereo_dets = extractRadarStereoFusedTracks(master_detections)
    OpenCV_radar_stereo_dets = extractRadarStereoFusedTracks(Opencv_detections)
    LaneVP_radar_stereo_dets = extractRadarStereoFusedTracks(laneVP_detections)
    target_radar_stereo_dets = extractRadarStereoFusedTracks(target_detections)

    plt.scatter(*getXrelDiff(master_radar_stereo_dets), marker='.', c='green', label='master RS#{}'.format(master_radar_stereo_dets.shape[0]))
    plt.scatter(*getXrelDiff(LaneVP_radar_stereo_dets), marker='.', c='blue', label='LaneVP RS#{}'.format(LaneVP_radar_stereo_dets.shape[0]))
    plt.scatter(*getXrelDiff(OpenCV_radar_stereo_dets), marker='.', c='pink', label='OpenCV RS#${}'.format(OpenCV_radar_stereo_dets.shape[0]))
    plt.scatter(*getXrelDiff(target_radar_stereo_dets), marker='.', c='gray', label='{} RS#${}'.format(target_name ,target_radar_stereo_dets.shape[0]))
    plt.legend()
    plt.xlabel('XrelRadar')
    plt.ylabel('XrelStereo - XrelRadar')
    plt.title(bag_name_master)
    plt.savefig("{}.jpg".format(bag_name_master))
    plt.close()
