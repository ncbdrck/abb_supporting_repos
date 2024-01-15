#! /usr/bin/env python
import rospy
import numpy as np
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import DBSCAN
import cv2 as cv
import struct
from computer_vision_pkg.msg import PosOriObjectsArray
from geometry_msgs.msg import Point, Quaternion
from math import pi

from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
import tf2_ros
from  tf.transformations import quaternion_from_euler, quaternion_from_matrix, euler_from_matrix

def publish_filtered_pointcloud(pointcloud, msg_header):
    processed_msg = PointCloud2()
    processed_msg.header = msg_header  # Copy the header from the received message
    processed_msg.height = 1
    processed_msg.width = len(pointcloud)
    processed_msg.fields = [
        pc2.PointField(name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1),
    ]
    processed_msg.point_step = 12  # 4 bytes for each field (x, y, z)
    processed_msg.row_step = processed_msg.point_step * processed_msg.width
    processed_msg.is_bigendian = False
    processed_msg.is_dense = True

    # Pack the processed points into the data field
    processed_msg_data = []
    for point in pointcloud:
        packed_data = struct.pack('fff', *point)
        processed_msg_data.extend(packed_data)

    processed_msg.data = processed_msg_data

    filtered_pc_publisher.publish(processed_msg)


def filter_raw_pointcloud(pointcloud):
  y_threshold_min = -0.9
  y_threshold_max = 0.32

  z_threshold_min = 0.002
  z_threshold_max = 0.5

  y_values = pointcloud[:,1]
  z_values = pointcloud[:,2]

  y_mask = np.logical_and(y_values > y_threshold_min, y_values < y_threshold_max)
  z_mask = np.logical_and(z_values > z_threshold_min, z_values < z_threshold_max)

  mask = np.logical_and(z_mask, y_mask)
  filtered_pointcloud = pointcloud[mask]
  return filtered_pointcloud


def get_object_segmentation(pointcloud, e=0.02, ms=800):
    eps = e  # Distance threshold for neighboring points
    min_samples = ms  # Minimum number of points in a cluster
    # Perform DBSCAN clustering
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    labels = dbscan.fit_predict(pointcloud)
    # Get unique labels assigned to each point
    unique_labels = np.unique(labels)
    # Segment the point cloud based on the unique labels
    segmented_point_cloud = []
    for label in unique_labels:
        segment = pointcloud[labels == label]
        segmented_point_cloud.append(segment)
    # Convert the segmented point clouds to NumPy arrays
    segmented_point_cloud = np.array([np.asarray(segment) for segment in segmented_point_cloud])
    # print(segmented_point_cloud)
    aux = []
    for pc in segmented_point_cloud:
      points = np.float32(pc)
      points = pc.reshape(len(pc), 3).astype('float64')
      if len(points) > 100:
        aux.append(points)
    return aux


def get_object_orientation(point_cloud, send_raw_data=False):
  part_orientation = []
  mean_array, eivec_array, pc1_array = [], [], []
  for pcloud in point_cloud:
    # points = np.float32(pc)
    # points = pc.reshape(len(pc), 3).astype('float64')
    mean, eivec, eival = cv.PCACompute2(pcloud, np.array([]))

    cntr = (mean[0, 0], mean[0, 1], mean[0, 2])
    p1 = np.array([cntr[0] + 0.1 * eivec[0, 0] * eival[0], cntr[1] + 0.1 * eivec[0, 1] * eival[0], cntr[2] + 0.1 * eivec[0, 2] * eival[0]])
    p2 = np.array([cntr[0] - 0.1 * eivec[1, 0] * eival[1], cntr[1] - 0.1 * eivec[1, 1] * eival[1], cntr[2] - 0.1 * eivec[1, 2] * eival[1]])

    t1, t2 = 600, 6000
    pc1 = [cntr[0] + t1 * (p1[0][0] - cntr[0]), cntr[1] + t1 * (p1[1][0] - cntr[1]), cntr[2] + t1 * (p1[2][0] - cntr[2])]
    pc2 = [cntr[0] + t2 * (p2[0][0] - cntr[0]), cntr[1] + t2 * (p2[1][0] - cntr[1]), cntr[2] + t2 * (p2[2][0] - cntr[2])]
    part_orientation.append([cntr, pc1, pc2])
    mean_array.append(mean[0])
    eivec_array.append(eivec) 
    pc1_array.append(pc1) 
  part_orientation_array = np.array(part_orientation)
  if send_raw_data:
    return [np.array(mean_array), np.array(eivec_array), np.array(pc1_array)]
  else:
    return part_orientation_array


def publish_object_position_array(object_array):
  mean_flatten = object_array[0].flatten()
  eivec_flatten = object_array[1].flatten()
  pc_flatten = object_array[2].flatten()
  print("\nMean values:\n", mean_flatten)
  print("\nEigenvector values:\n", eivec_flatten)
  print("\nPrincipal component values:\n", pc_flatten)
  object_pos_array = PosOriObjectsArray()
  object_pos_array.numObj = len(object_array[0])
  object_pos_array.meanArray = mean_flatten
  object_pos_array.eivecArray = eivec_flatten
  object_pos_array.pcArray = pc_flatten
  rospy.loginfo("============== Publishing the MEAN and EIGENVECTOR for %s objects found ==============", len(object_array[0]))
  object_array_publisher.publish(object_pos_array)
  rospy.loginfo("============== The object data was published! ==============")


def callback(data):
  # Accessing the point cloud data
  points = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
  points_array = np.array(list(points))
  if len(points_array) > 0:
    filtered_pointcloud = filter_raw_pointcloud(points_array)
    publish_filtered_pointcloud(filtered_pointcloud, data.header)
    if len(filtered_pointcloud) > 0:
      rospy.loginfo("============== Segmenting pointcloud ==============")
      segmented_point_cloud = get_object_segmentation(filtered_pointcloud)
      rospy.loginfo("============== Number of objects detected: %s ==============", len(segmented_point_cloud))
      mean, eivec, pc1 = get_object_orientation(segmented_point_cloud, send_raw_data=True)
      
      # pc_array = get_object_orientation(segmented_point_cloud)
      # fig = plt.figure()
      # ax = fig.add_subplot(111, projection='3d')
      # for i, pc in enumerate(segmented_point_cloud):
      #   ax.scatter(0,0,0.49066,s=5,c="green")
      #   ax.scatter(pc[:, 0],pc[:, 1],pc[:, 2],s=0.01,c="red",alpha=0.5)
      #   ax.scatter(pc_array[i][0][0], pc_array[i][0][1], pc_array[i][0][2],s=20,c="red")
      #   ax.scatter(pc_array[i][1][0], pc_array[i][1][1], pc_array[i][1][2],s=20,c="blue")
      #   ax.scatter(pc_array[i][2][0], pc_array[i][2][1], pc_array[i][2][2],s=20,c="green")
      #   ax.plot([pc_array[i][0][0], pc_array[i][1][0]], [pc_array[i][0][1], pc_array[i][1][1]], [pc_array[i][0][2], pc_array[i][1][2]], color='blue')
      #   ax.plot([pc_array[i][0][0], pc_array[i][2][0]], [pc_array[i][0][1], pc_array[i][2][1]], [pc_array[i][0][2], pc_array[i][2][2]], color='green')
      # ax.set_xlabel('X')
      # ax.set_ylabel('Y')
      # ax.set_zlabel('Z')
      # ax.set_title('Point Cloud Visualization')
      # plt.show()
      publish_object_position_array([mean, eivec, pc1])



def main():
  global filtered_pc_publisher
  global object_array_publisher
  rospy.init_node('pointcloud_topic_subscriber')
  rospy.Subscriber("/phoxi_camera/pointcloud", sensor_msgs.msg.PointCloud2, callback)
  filtered_pc_publisher = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=10)
  object_array_publisher = rospy.Publisher('object_pos_array_topic', PosOriObjectsArray, queue_size=10)
  rospy.spin()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass









#########################################################################################################################


            # x = new_points[:, 0]
            # y = new_points[:, 1]
            # z = new_points[:, 2]
            # fig = plt.figure()
            # ax = fig.add_subplot(111, projection='3d')
            # # ax.scatter(x, y, z, s=0.1)
            # ax.scatter(0,0,0.49066,s=5,c="green")
            # ax.scatter(0,0,0,s=5,c="red")
            # for i, pc in enumerate(segmented_point_cloud):
            #     ax.scatter(pc[:, 0],pc[:, 1],pc[:, 2],s=0.01,c="red",alpha=0.5)
            #     ax.scatter(pc_array[i][0][0], pc_array[i][0][1], pc_array[i][0][2],s=20,c="red")
            #     ax.scatter(pc_array[i][1][0], pc_array[i][1][1], pc_array[i][1][2],s=20,c="blue")
            #     ax.scatter(pc_array[i][2][0], pc_array[i][2][1], pc_array[i][2][2],s=20,c="green")
            #     ax.plot([pc_array[i][0][0], pc_array[i][1][0]], [pc_array[i][0][1], pc_array[i][1][1]], [pc_array[i][0][2], pc_array[i][1][2]], color='blue')
            #     ax.plot([pc_array[i][0][0], pc_array[i][2][0]], [pc_array[i][0][1], pc_array[i][2][1]], [pc_array[i][0][2], pc_array[i][2][2]], color='green')

            # ax.set_xlabel('X')
            # ax.set_ylabel('Y')
            # ax.set_zlabel('Z')
            # ax.set_title('Point Cloud Visualization')
            # plt.show()