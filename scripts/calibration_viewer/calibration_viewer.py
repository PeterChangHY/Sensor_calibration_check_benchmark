#!/usr/bin/env python

import sys
import argparse
import cv2
from cv_bridge import CvBridge
import os
import numpy as np
import logging
import pypcd
import matplotlib.pyplot as plt
import operator

# add system path for local packages and moudles
try:
    path = os.path.normpath(os.path.join(os.path.abspath(__file__), '../../'))
    sys.path.insert(0, path)
except ValueError:
    print('Can not inser the path {}'.format(path))
# import from local common package
from common import rotate_image
from common import load_bag
from common import parse_radar_message
from common import project_points
from common import calib
from common import golden_lane
from common import horizontal_image_stitcher
from common import cylindrical_image_stitcher

bridge = CvBridge()
logger = logging.getLogger(__name__)
# define lidar projection color BGR
lidar_color = [(0, 255, 0), (128, 0, 128), (255, 0, 0), (0, 255, 128), (128, 255, 0)]
radar_color = [(255, 100, 100), (0, 100, 128), (100, 50, 180), (0, 0, 255), (50, 50, 100)]

image_name_pattern = "%04d.png"
lidar_name_pattern = "%04d.pcd"

args = None


class CalibratonViewer(object):
    def __init__(self, cam_calibs, lidar_calibs, radar_calibs, output_dir):
        self.cam_calibs = cam_calibs
        self.lidar_calibs = lidar_calibs
        self.radar_calibs = radar_calibs
        self.output_dir = output_dir
        self.horizontal_image_stitcher = horizontal_image_stitcher.HorizontalImageStitcher(
            height=1020)
        # self.birdeyeview_image_stitchers = {}
        self.cylindrical_image_stitchers = {}

        all_sensor_topics = [topic for topic in cam_calibs] + [topic for topic in lidar_calibs]
        self.ensure_output_directories(all_sensor_topics)

        for topic in cam_calibs:
            cam_calib = cam_calibs[topic]
            Tr_cam_to_imu = cam_calib.Tr_cam_to_imu
            projection_matrix = cam_calib.P
            src_shape = [cam_calib.height, cam_calib.width]
            cylindrical_dst_shape = [1020, 3000]
            # self.birdeyeview_image_stitchers[topic] = birdeyeview_image_stitcher.BirdEyeViewImageStitcher(
            #    Tr_imu_to_cam, imu_height, projection_matrix, src_shape, birdeyeview_dst_shape, scale)
            self.cylindrical_image_stitchers[topic] = cylindrical_image_stitcher.CylindricalImageStitcher(
                Tr_cam_to_imu, projection_matrix, src_shape, cylindrical_dst_shape)

    def load_messages(self, cam_messages, lidar_messages, radar_messages):

        # load all images
        images = {cam_msg.topic: bridge.compressed_imgmsg_to_cv2(
            cam_msg.message, desired_encoding="passthrough") for cam_msg in cam_messages}
        for topic in images:
            image = images[topic]
            image = cv2.putText(image, topic.split(
                '/')[1], (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        # load all lidar messages

        # fixed the wrong  fileds of unified points to XYZIRT
        for lidar_msg in lidar_messages:
            corrected_fields = lidar_msg.message.fields[:6]
            corrected_fields[4].name = 'ring'
            corrected_fields[4].offset = 16
            corrected_fields[4].datatype = 4
            corrected_fields[5].name = 'timstamp'
            corrected_fields[5].offset = 18
            corrected_fields[5].datatype = 8
            lidar_msg.message.fields = corrected_fields

        pointclouds = {lidar_msg.topic: pypcd.PointCloud.from_msg(lidar_msg.message)
                       for lidar_msg in lidar_messages}

        # load all radar messages
        radar_tracks = {radar_msg.topic: parse_radar_message.parse_radar_track(
            radar_msg.message) for radar_msg in radar_messages}

        return images, pointclouds, radar_tracks

    def write_images(self, images, frame_number, suffix=None):
        image_basename = image_name_pattern % frame_number
        if suffix:
            image_basename = image_basename[:-4] + suffix + image_basename[-4:]
        for topic in images:
            image = images[topic]
            cv2.imwrite(os.path.join(self.get_topic_outdir(topic), image_basename), image)

    def write_pointclouds(self, pointclouds, frame_number):
        lidar_basename = lidar_name_pattern % frame_number
        for topic in pointclouds:
            pointcloud = pointclouds[topic]
            pointcloud.save(os.path.join(self.get_topic_outdir(topic), lidar_basename))

    def rectify_images(self, images):
        rect_images = {}
        for topic in images:
            image = images[topic]
            cam_calib = self.cam_calibs[topic]
            print(topic,cam_calib.upside_down)
            if cam_calib.upside_down == 1:
                image = cv2.flip(image, 0)
                #cv2.imshow('show', image)
                #cv2.waitKey(0)
            rect_images[topic] = cam_calib.rectify(image)
        return rect_images

    def project_lidar_to_rect_images(self, pointclouds, images):
        for topic in images:
            cam_calib = self.cam_calibs[topic]
            draw_image = images[topic]
            Tr_lidar_to_imu = None
            color_index = 0
            for lidar_topic in pointclouds:
                pointcloud = pointclouds[lidar_topic]
                lidar_calib = self.lidar_calibs[lidar_topic]
                Tr_lidar_to_imu = lidar_calib.Tr_lidar_to_imu
                Tr_lidar_to_cam = cam_calib.Tr_imu_to_cam.dot(Tr_lidar_to_imu)

                cloud_data = pointcloud.pc_data
                xyz_data = np.array([[data[0], data[1], data[2]] for data in cloud_data])

                proj_points = project_points.project_point(
                    Tr_lidar_to_cam, cam_calib.P, (cam_calib.width, cam_calib.height), xyz_data)
                color_bgr = lidar_color[color_index % len(lidar_color)]
                for pt in proj_points:
                    draw_image = cv2.circle(draw_image, pt, 1, color_bgr, -1)

                draw_image = cv2.putText(draw_image, lidar_topic.split('/')[1], (20, 50 + 30 * color_index), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                         color_bgr, 2, cv2.LINE_AA)
                color_index += 1

    def project_radar_to_rect_images(self, all_radar_tracks, images):
        for topic in images:
            cam_calib = self.cam_calibs[topic]
            draw_image = images[topic]
            color_index = 0
            for radar_topic in all_radar_tracks:
                radar_tracks = all_radar_tracks[radar_topic]
                radar_calib = self.radar_calibs[radar_topic]
                Tr_radar_to_imu = radar_calib.Tr_radar_to_imu
                Tr_radar_to_cam = cam_calib.Tr_imu_to_cam.dot(Tr_radar_to_imu)
                color_bgr = radar_color[color_index % len(radar_color)]
                for radar_track in radar_tracks:
                    xyz_data = []
                    # assume lidar tracks are at the ground
                    imu_height = 1.3
                    radar_obstacle_contours = np.array(
                        [[p[0], p[1], -imu_height] for p in radar_track['point']])
                    xyz_data.append(np.mean(radar_obstacle_contours, axis=0))

                    proj_point = project_points.project_point(
                        Tr_radar_to_cam, cam_calib.P, (cam_calib.width, cam_calib.height), np.array(xyz_data))

                    if len(proj_point) > 0:
                        draw_image = cv2.ellipse(
                            draw_image, proj_point[0], (10, 60), 0, 0, 360, color_bgr, 3)
                        draw_image = cv2.putText(draw_image, 'id: ' + str(radar_track['track_id']), proj_point[0], cv2.FONT_HERSHEY_SIMPLEX,
                                                 1.0, color_bgr, 1, cv2.LINE_AA)

                draw_image = cv2.putText(draw_image, radar_topic.split('/')[1], (draw_image.shape[1] - 500, 20 + 30 * color_index), cv2.FONT_HERSHEY_SIMPLEX,
                                         1, color_bgr, 2, cv2.LINE_AA)
                color_index += 1

    def draw_golden_lane(self, rect_images):
        for topic in rect_images:
            cam_calib = self.cam_calibs[topic]
            imu_to_image = cam_calib.P.dot(cam_calib.Tr_imu_to_cam)
            rect_images[topic] = golden_lane.draw_golden_lane(
                rect_images[topic], start=-30.0, curvature=0, length=80.0, lane_width=3.8, imu_height=1.3, imu_to_image=imu_to_image)

    def rotate_image_by_calib(self, image, calib):
        # rotate image 90, 180 or 270 degree, depend on the camera's y axis in the imu's coordinate
        # project the camera's y axis on imu's ZY plane and get the angle between the projected y axis the positive imu's z axis
        # when imu's Z axis is point up and the camera's y axis's point down then we don't need to rotate the image
        theta = np.arctan2(calib.Tr_cam_to_imu[1, 1], calib.Tr_cam_to_imu[2, 1])
        new_image = np.copy(image)
        logger.info(calib.sensor_name)
        if theta > 1.4 and theta < 1.7:
            # rotate 90 degree
            logger.info("theta: {}, rotate 90 degree".format(theta))
            new_image = rotate_image.rotate_image(image, 90)
        if theta < -1.4 and theta > -1.7:
            # rotate 270 degree
            logger.info("theta: {}, rotate 270 degree".format(theta))
            new_image = rotate_image.rotate_image(image, 270)
        if abs(theta) < 0.1:
            # rotate 180 degree
            logger.info("theta: {}, rotate 180 degree".format(theta))
            new_image = rotate_image.rotate_image(image, 180)

        return new_image

    def rotate_images_by_calib(self, rect_images):
        rotated_images = {}
        for topic in rect_images:
            rect_image = rect_images[topic]
            rotated_images[topic] = self.rotate_image_by_calib(rect_image, self.cam_calibs[topic])
        return rotated_images

    def generate_horizontal_stitched_image(self, rect_images):
        # change the order of images based on ca,era's orientation, so we can get a panoramic-like stitched image
        phi_dic = {}
        # calculate the camera's yaw in imu's coordinate
        for topic in self.cam_calibs:
            cam_calib = cam_calibs[topic]
            phi = np.arctan2(cam_calib.Tr_cam_to_imu[1, 2], cam_calib.Tr_cam_to_imu[0, 2])
            phi_dic[topic] = phi
        # sort by yaw
        phi_dic = sorted(phi_dic.items(), key=operator.itemgetter(1), reverse=True)
        phi_dic = dict(phi_dic)
        sorted_images = []
        for topic in phi_dic:
            sorted_images.append(rect_images[topic])

        return self.horizontal_image_stitcher.horizontal_stitch(sorted_images, args.draw_epipolar_line)

    def generate_cylindrical_stitched_image(self, rect_images):
        dst_shape = self.cylindrical_image_stitchers.values()[0].dst_shape
        cylindrical_image = np.zeros((dst_shape[0], dst_shape[1], 3), dtype=np.uint8)

        for topic in rect_images:
            image = rect_images[topic]
            image_stitcher = self.cylindrical_image_stitchers[topic]
            image[0, 0, :] = [0, 0, 0]  # a hack to set non-map pixel to black
            dst_image = image_stitcher.remap(image)
            cylindrical_image = cv2.addWeighted(cylindrical_image, 1.0, dst_image, 0.5, 0)

        return cylindrical_image

    def generate_birdeyeview_stitched_image(self, rect_images):
        dst_shape = self.birdeyeview_image_stitchers.values()[0].dst_shape
        birdeyeview_image = np.zeros((dst_shape[0], dst_shape[1], 3), dtype=np.uint8)

        for topic in rect_images:
            image = rect_images[topic]
            image_stitcher = self.birdeyeview_image_stitchers[topic]
            image[0, 0, :] = [0, 0, 0]  # a hack to set non-map pixel to black
            dst_image = image_stitcher.remap(image)
            birdeyeview_image = cv2.addWeighted(birdeyeview_image, 1.0, dst_image, 0.5, 0)
        scale = self.birdeyeview_image_stitchers.values()[0].scale
        height, width = dst_shape
        # draw grid
        for i in range(-10, 15, 5):
            # horizontal line
            cv2.line(birdeyeview_image, (0, int(i * scale + height / 2)),
                     (width, int(i * scale + height / 2)), (0, 0, 255), 1)
        for i in range(-100, 105, 5):
            # vertical line
            cv2.line(birdeyeview_image, (int(i * scale + width / 2), 0),
                     (int(i * scale + width / 2), height), (0, 0, 255), 1)

        cv2.putText(birdeyeview_image, "0,0", (int(width / 2), int(height / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
        cv2.putText(birdeyeview_image, "5m", (int(width / 2), int(height / 2 + 5 * scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
        cv2.putText(birdeyeview_image, "10m", (int(width / 2), int(height / 2 + 10 * scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
        cv2.putText(birdeyeview_image, "5m", (int(width / 2 + 5 * scale), int(height / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
        cv2.putText(birdeyeview_image, "50m", (int(width / 2 + 50 * scale), int(height / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
        cv2.putText(birdeyeview_image, "100m", (int(width / 2 + 100 * scale), int(height / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
        cv2.putText(birdeyeview_image, "-50m", (int(width / 2 + -50 * scale), int(height / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
        cv2.putText(birdeyeview_image, "-100m", (int(width / 2 + -100 * scale), int(height / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))

        return birdeyeview_image

    def draw_lidar_in_topdown_view(self, pointclouds, plot_ax):
        Tr_lidar_to_imu = None
        lidar_index = 0
        for lidar_topic in pointclouds:
            pc = pointclouds[lidar_topic]
            lidar_calib = self.lidar_calibs[lidar_topic]
            Tr_lidar_to_imu = lidar_calib.Tr_lidar_to_imu

            cloud_data = pc.pc_data
            xyz_data = np.array([[data[0], data[1], data[2]] for data in cloud_data])

            xyz_data_imu = project_points.transform_point(Tr_lidar_to_imu, xyz_data)

            color_bgr = lidar_color[lidar_index % len(lidar_color)]
            color_rgb = [k / 255.0 for k in reversed(color_bgr)]
            plot_ax.scatter(xyz_data_imu[:, 0], xyz_data_imu[:, 1],
                            color=color_rgb, marker=',', s=1.0, alpha=0.4, label=lidar_topic)

            lidar_index += 1

    def draw_radar_in_topdown_view(self, all_radar_tracks, plot_ax):
        radar_index = 0

        for topic in all_radar_tracks:
            radar_tracks = all_radar_tracks[topic]
            radar_calib = self.radar_calibs[topic]
            Tr_radar_to_imu = radar_calib.Tr_radar_to_imu
            # radar_tracks = parse_radar_message.parse_radar_track(radar_msg.message)
            color_bgr = radar_color[radar_index % len(radar_color)]
            color_rgb = [k / 255.0 for k in reversed(color_bgr)]
            xyz_data = []
            velocity_data = []
            for radar_track in radar_tracks:
                radar_obstacle_contours = np.array(
                    [[p[0], p[1], p[2]] for p in radar_track['point']])
                mean_xyz = np.mean(radar_obstacle_contours, axis=0)
                xyz_data.append(mean_xyz)
                velocity_data.append(np.array(radar_track['velocity']))
                # xyz_data = np.array(xyz_data)
            xyz_data_imu = project_points.transform_point(Tr_radar_to_imu, np.array(xyz_data))
            velocity_data_imu = project_points.transform_vector(Tr_radar_to_imu, np.array(velocity_data))
            if xyz_data_imu.any():
                plot_ax.scatter(xyz_data_imu[:, 0], xyz_data_imu[:, 1], color=color_rgb,
                                marker='+', s=20.0, alpha=0.8, label=topic)
                plot_ax.quiver(xyz_data_imu[:, 0], xyz_data_imu[:, 1], velocity_data_imu[:, 0], velocity_data_imu[:, 1], color=color_rgb)

            radar_index += 1

    def write_topdown_view(self, pointclouds, radar_tracks, frame_number):
        fig, bird_eye_ax = plt.subplots()
        self.draw_lidar_in_topdown_view(pointclouds, bird_eye_ax)
        self.draw_radar_in_topdown_view(radar_tracks, bird_eye_ax)

        bird_eye_ax.set_xlim((-100, 200))
        bird_eye_ax.set_ylim((-60, 60))
        bird_eye_ax.set_xticks(np.arange(-100, 200, step=10))
        bird_eye_ax.set_yticks(np.arange(-60, 60, step=5))
        bird_eye_ax.tick_params(axis='both', which='major', labelsize=7)
        bird_eye_ax.set_xlabel('x(meter)', fontsize=10)
        bird_eye_ax.set_ylabel('y(meter)', fontsize=10)
        bird_eye_ax.grid()
        bird_eye_ax.legend(loc='upper left', fontsize=7, framealpha=0.5)

        image_name = image_name_pattern % frame_number
        plt.savefig(os.path.join(self.output_dir, 'topdown', image_name), dpi=250)
        plt.close('all')

    def write_frame(self, frame_number, cam_messages, lidar_messages, radar_messages):

        images, pointclouds, all_radar_tracks = self.load_messages(
            cam_messages, lidar_messages, radar_messages)

        self.write_images(images, frame_number)
        self.write_pointclouds(pointclouds, frame_number)

        rect_images = self.rectify_images(images)

        if args.project_lidar_to_image:
            self.project_lidar_to_rect_images(pointclouds, rect_images)

        if args.project_radar_to_image:
            self.project_radar_to_rect_images(all_radar_tracks, rect_images)

        if args.draw_golden_lane:
            self.draw_golden_lane(rect_images)

        self.write_images(rect_images, frame_number, suffix='_rect')

        rotated_rect_images = self.rotate_images_by_calib(rect_images)

        horizontal_stitched_image = self.generate_horizontal_stitched_image(rotated_rect_images)
        self.write_images({'horizontal_stitched_image': horizontal_stitched_image}, frame_number)

        cylindrical_stitched_image = self.generate_cylindrical_stitched_image(rect_images)
        self.write_images({'cylindrical_stitched_image': cylindrical_stitched_image}, frame_number)

        # birdeyeview_stitched_image = self.generate_birdeyeview_stitched_image(rect_images)
        # self.write_images({'birdeyeview_stitched_image': birdeyeview_stitched_image}, frame_number)

        self.write_topdown_view(pointclouds, all_radar_tracks, frame_number)

    def get_topic_outdir(self, topic):
        if len(topic.split("/")) > 1:
            return os.path.join(self.output_dir, topic.split("/")[1])
        else:
            return os.path.join(self.output_dir, topic)

    def ensure_output_directories(self, all_sensor_topics):
        if not os.path.exists(self.output_dir):
            os.mkdir(self.output_dir)

        for sensor_topic in all_sensor_topics:
            sensor_dir = self.get_topic_outdir(sensor_topic)
            if not os.path.isdir(sensor_dir):
                print("creating ", sensor_dir)
                os.mkdir(sensor_dir)

        others = ['topdown', 'horizontal_stitched_image',
                  'cylindrical_stitched_image']

        for other in others:
            other_dir = os.path.join(self.output_dir, other)
            if not os.path.isdir(other_dir):
                print("creating ", other_dir)
                os.mkdir(other_dir)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags", required=True, type=str, metavar='BAG', nargs='+',
                        help="path to bag (can be more than one, separated by space)")
    parser.add_argument("--rate", type=float, default=0.1,
                        help="how often to dump a message, in seconds")
    parser.add_argument("--tolerance", type=float, default=0.05,
                        help="max gap to allow between message pairs")
    parser.add_argument("--cam_topics", type=str, metavar='cam_topic', nargs='+',
                        help="camera topics, split by comma or space", default="/front_left_camera/image_color/compressed")
    parser.add_argument("--stereo_topics", type=str, metavar='stereo_topic', nargs=2,
                        help="stereo topics, split by space", default=[])
    parser.add_argument("--stereo_calib", type=str, metavar='mono_cam_calib',
                        nargs=1, help="stereo calibration file")
    parser.add_argument("--cam_calibs", type=str, metavar='mono_cam_calib',
                        nargs='+', help="camera calibration files, split by space")
    parser.add_argument("--lidar_topics", type=str, metavar='lidar_topic',
                        nargs='*', help="lidar topics, split by comma or space")
    parser.add_argument("--lidar_calibs", type=str, metavar='lidar_calib',
                        nargs='*', help="lidar/sidelidar calibration files, split by space")
    parser.add_argument("--radar_topics", type=str, metavar='radar_topic',
                        nargs='*', help="radar tracks topic, split by comma or space")
    parser.add_argument("--radar_calibs", type=str, metavar='radar_calib',
                        nargs='*', help="radar calibration files, split by space")
    parser.add_argument("--project_lidar_to_image", action='store_true',
                        help="display lidar point on image when topic is available")
    parser.add_argument("--project_radar_to_image", action='store_true',
                        help="display radar point on image when topic is available")
    parser.add_argument("--draw_golden_lane", action='store_true',
                        help="display golden lane in all rectified images")
    parser.add_argument("--draw_epipolar_line", action='store_true',
                        help="display epipolar in horizontal stitched images")
    parser.add_argument("--output_dir", type=str, default='.', help="output directory")

    args = parser.parse_args()

    # log config
    logging.basicConfig(filename='calibration_viewer.log', format='%(asctime)s.%(msecs)03d %(levelname)s:%(message)s',
                        datefmt='%Y-%m-%d,%H:%M:%S', level=logging.DEBUG)

    def load_topic_with_calib(topics, calib_files):
        if topics is None or len(topics) == 0 or calib_files is None or len(calib_files) == 0:
            return {}
        result = {}
        assert len(topics) == len(calib_files)
        for index in range(len(topics)):
            calib_file = calib_files[index]
            topic = topics[index]
            calib_obj = calib.load_calib(calib_file)
            if calib_obj is None:
                print('Cannot find calib_file: %s' % calib_file)
            result[topic] = calib_obj
        return result

    # load camera topics and calibs
    cam_calibs = load_topic_with_calib(args.cam_topics, args.cam_calibs)

    lidar_calibs = load_topic_with_calib(args.lidar_topics, args.lidar_calibs)

    radar_calibs = load_topic_with_calib(args.radar_topics, args.radar_calibs)

    all_topics = []
    all_topics += [topic for topic in cam_calibs]
    all_topics += [topic for topic in lidar_calibs]
    all_topics += [topic for topic in radar_calibs]


    ros_bags = load_bag.load_rosbags_from_files(args.bags)
    missing_topic = False
    for topic in all_topics:
        if not load_bag.check_topic_exist_in_bag(ros_bags[0], topic):
            missing_topic = True
    
    if missing_topic:
        sys.exit(1)
    
    calib_viewer = CalibratonViewer(cam_calibs, lidar_calibs, radar_calibs, args.output_dir)
    
    bag_it = load_bag.buffered_message_generator(ros_bags, all_topics, args.tolerance)
    
    prev_frame_number = -1
    start_time = None
    for frame in bag_it:
        camera_messages = [frame[topic] for topic in cam_calibs]
        lidar_messages = [frame[topic] for topic in lidar_calibs]
        radar_messages = [frame[topic] for topic in radar_calibs]
        if start_time is None:
            start_time = camera_messages[0].message.header.stamp
        frame_number = int(
            ((camera_messages[0].message.header.stamp - start_time).to_sec() + (args.rate / 2.0)) / args.rate)
        if frame_number == prev_frame_number:
            print("oddly, duplicated frame")
            continue
        if frame_number % 10 == 0:
            print("frame %d" % (frame_number))
        if frame_number - prev_frame_number > 1:
            print("warning, skipped %d frames " % (frame_number - prev_frame_number - 1))
        calib_viewer.write_frame(frame_number, camera_messages, lidar_messages, radar_messages)

        prev_frame_number = frame_number
