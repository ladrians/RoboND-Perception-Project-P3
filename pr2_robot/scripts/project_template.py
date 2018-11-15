#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


## Code reorganized

def statistical_filter(pcl_msg):
    outlier_filter = pcl_msg.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 1.0
    outlier_filter.set_std_dev_mul_thresh(x)
    sts_filter = outlier_filter.filter()
    return sts_filter

def voxel_downsample(pcl_msg):
    vox = pcl_msg.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    return cloud_filtered

def passthrough_filter(pcl_msg, filter_axis, axis_min, axis_max):
    passthrough = pcl_msg.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    return cloud_filtered

def ransac_segmentation(pcl_msg):
    seg = pcl_msg.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    return seg

def euclidean_cluster(outlier):
    white_cloud = XYZRGB_to_XYZ(outlier)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02) # 2 cm
    ec.set_MinClusterSize(100) # parameters values ideas from http://pointclouds.org/documentation/tutorials/cluster_extraction.php
    ec.set_MaxClusterSize(250000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    return white_cloud, cluster_indices

def segment_cluster_by_color(white_cloud, cluster_indices):
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    return cluster_cloud

def histogram_feature_vector(sample_cloud):
    # Extract histogram features
    hists = compute_color_histograms(sample_cloud, using_hsv=True)
    normals = get_normals(sample_cloud)
    nhists = compute_normal_histograms(normals)

     # Compute the associated feature vector
    feature = np.concatenate((hists, nhists))
    return feature

def get_scene_number():
    scene_number = rospy.get_param('/scene_number')
    num = int(scene_number)
    return num

def get_object_group(obj_name, object_list_param):
    object_group = ''
    for item in object_list_param:
        if item['name'] == obj_name:
            object_group = item['group']
            break
    return object_group

def get_arm_name(object_group):
    name = 'left' if object_group == 'red' else 'right'
    return name

def get_place_pose(object_group, dropbox_list_param):
    pose = Pose()
    for item in dropbox_list_param:
        if item['group'] == object_group:
            position = item['position']
            pose.position.x = position[0]
	    pose.position.y = position[1]
	    pose.position.z = position[2]
    return pose

# Callback function for your Point Cloud Subscriber
def pcl_callback(ros_pcl_msg):

# Exercise-2:

    # Convert ROS msg to PCL data
    pcl_msg = ros_to_pcl(ros_pcl_msg)
    
    # Statistical Outlier Filtering
    sts_filter = statistical_filter(pcl_msg)

    # Voxel Grid Downsampling
    voxel_filtered = voxel_downsample(sts_filter)

    # PassThrough Filters
    pass_filter = passthrough_filter(voxel_filtered, 'z', 0.6, 1.1) # Table filter
    pass_filter = passthrough_filter(pass_filter, 'x', 0.3, 4) # Object filter

    # RANSAC Plane Segmentation
    seg = ransac_segmentation(pass_filter)

    # Extract inliers and outliers
    inliers, coefficients = seg.segment()
    extracted_inliers = pass_filter.extract(inliers, negative=False) # Table
    extracted_outliers = pass_filter.extract(inliers, negative=True) # Objects

    # Euclidean Clustering
    white_cloud, cluster_indices = euclidean_cluster(extracted_outliers)

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_cloud = segment_cluster_by_color(white_cloud, cluster_indices)

    # Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_cloud_table = pcl_to_ros(extracted_inliers)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cluster_cloud)
    pcl_table_pub.publish(ros_cloud_table)

# Exercise-3:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = extracted_outliers.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        object_cloud = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        feature = histogram_feature_vector(object_cloud)

        # Make the prediction, retrieve the label for the result and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = object_cloud
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException as e:
        #pass
        rospy.loginfo('Exception {}:{}'.format(e.name, e.description))

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    test_scene_num = Int32()
    test_scene_num.data = get_scene_number()
    yaml_filename = 'output_%s.yaml'%test_scene_num.data
    dict_list = []

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')


    # Parse parameters into individual variables
    for obj in object_list:

        obj_name = str(obj.label)

        # Loop through the pick list
        object_group = get_object_group(obj_name, object_list_param)

        # TODO: Rotate PR2 in place to capture side tables for the collision map

        # Get the PointCloud for a given object and obtain it's centroid
        points_arr = ros_to_pcl(obj.cloud).to_array()
        centroid = np.mean(points_arr, axis=0)[:3]
        center_x = np.asscalar(centroid[0])
        center_y = np.asscalar(centroid[1])
        center_z = np.asscalar(centroid[2])

        pick_pose.position.x = center_x
        pick_pose.position.y = center_y
        pick_pose.position.z = center_z

        # Create 'place_pose' for the object
        place_pose = get_place_pose(object_group, dropbox_list_param)

        # Assign the arm to be used for pick_place
        arm_name.data = get_arm_name(object_group)

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        object_name.data = obj_name
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        '''
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        '''

    # Output your request parameters into output yaml file
    send_to_yaml(yaml_filename, dict_list)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('object_recognition', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

