## Project: Perception Pick & Place


My solution for the Robotics NanoDegree [Project #3](https://github.com/udacity/RoboND-Perception-Project).

[//]: # (Image References)

[image1]: ./misc_images/confusion_matrix_1.png
[image2]: ./misc_images/confusion_matrix_2.png
[image3]: ./misc_images/sample_1.png
[image4]: ./misc_images/sample_2.png
[image5]: ./misc_images/sample_3.png
[image6]: ./misc_images/segmentation_1.png
[image7]: ./misc_images/table_filter_1.png
[image8]: ./misc_images/object_filter_1.png

---
### Description

The objective is to construct a Perception pipeline based on the [Perception lessons](https://github.com/udacity/RoboND-Perception-Exercises); to recognize specific objects on the scene and it's location. As a secondary objective is to pick and place them in the correct bin.

### Pre-Configuration

Initially the training model from the lecture was tested but the results were not good so the pre-requisite was to extract again features for all the objects and retrain the SVM model.

1. Feature extraction

For each object, take 70 attempts to get a valid point cloud then extract histogram features, for more information check the [capture_features.py](pr2_robot/scripts/capture_features.py) file.

The object selection is as follows:

```python
# test 3
models = [\
   'sticky_notes',
   'book',
   'snacks',
   'biscuits',
   'eraser',
   'soap2',
   'soap',
   'glue']
```

To launch the capture phase execute the following ([complete perception exercises](https://github.com/udacity/RoboND-Perception-Exercises) are needed because of the `sensor_stick` package usage):

```ros
roslaunch sensor_stick training.launch
rosrun pr2_robot capture_features.py
```

It will create a `training_set.sav` file on the current folder ([training_set.sav](training_set.sav)).

2. Model training using a SVM model.

Based on the previous file using a linear SVM model will generate a clasifier on the [model.sav](model.sav) file to be used later, for more information check the [train_svm.py](pr2_robot/scripts/train_svm.py) file.

```ros
rosrun pr2_robot train_svm.py
```

The result regarding the classifier overall accuracy is:

```
Features in Training Set: 560
Invalid Features in Training set: 1
559
Scores: [ 0.99107143  0.96428571  0.99107143  1.          0.96396396]
Accuracy: 0.98 (+/- 0.03)
accuracy score: 0.982110912343
```

Two confusion matrices are detailed as follows; showing two different versions of the confusion matrix for the classifier.

The raw counts:

![confusion matrix without normalization][image1]

Percentage of the total:

![confusion matrix normalized][image2]

The objects were spawned randomly aroung 70 times.

Note: the sklearn library was updated to the 0.2 version so some modifications were done on the code.

### Launch-Configuration

There are 3 scenarios to check (`test1-3.world` in `/pr2_robot/worlds/`); a modification was done on the [pick_place_project.launch](pr2_robot/launch/pick_place_project.launch) to easily select the desired scenario, use the following argument to change the scene.

```xml
<arg name="scene_number" default="1" /> <!-- 1 | 2 | 3 -->
```

Check the `Results` section for more detail.

### Pipeline

The [project_template.py](pr2_robot/scripts/project_template.py) details the pipeline created using basic data from previous Exercises.

Subscribe to the camera data (point cloud) topic `/pr2/world/points` and start the pipeline on the `pcl_callback` function.

#### Filtering

##### Remove Noise

The first step is to remove noise from the dataset; to clean it up the first filter is a statistical Outlier Filtering.

```python
sts_filter = statistical_filter(pcl_msg)
```

##### Downsampling

The second step is to downsample the data to improve performance and remove unnecesary complexity using a voxel grid.

```python
voxel_filtered = voxel_downsample(sts_filter)
```

##### PassThrough Filters

Then, filter further the region of interes on the z-axis and x-axis to isolate the table and objects respectively.

```python
pass_filter = passthrough_filter(voxel_filtered, 'z', 0.6, 1.1) # Table filter
pass_filter = passthrough_filter(pass_filter, 'x', 0.3, 4) # Object filter
```
Table filter sample:

![Table filter Sample][image7]

Object filter sample:

![Object filter Sample][image8]

#### Segmentation

Next, used RANSAC plane fitting to segment the table in the scene; to separate the objects from the scene.

```python
seg = ransac_segmentation(pass_filter)
```

Extract inliers and outliers

```python
extracted_inliers = pass_filter.extract(inliers, negative=False) # Table
extracted_outliers = pass_filter.extract(inliers, negative=True) # Objects
```

#### Clustering

The Euclidean Clustering technique separates the objects into distinct clusters, thus completing the segmentation process.

```python
white_cloud, cluster_indices = euclidean_cluster(extracted_outliers)
...
cluster_cloud = segment_cluster_by_color(white_cloud, cluster_indices)
```

#### Object Recognition

Depending on the scene there are different objects to be identified, they will be labeled accordingly.

The first step is to classify the detected clusters using the SVM linear model based on the histogram features and make a prediction.

```python
feature = histogram_feature_vector(object_cloud)
...
prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
label = encoder.inverse_transform(prediction)[0]
```

The `histogram_feature_vector` function is implemented as follows in the same file using the exercises suggestion:

```python
def histogram_feature_vector(sample_cloud):
    # Extract histogram features
    hists = compute_color_histograms(sample_cloud, using_hsv=True)
    normals = get_normals(sample_cloud)
    nhists = compute_normal_histograms(normals)

    # Compute the associated feature vector
    feature = np.concatenate((hists, nhists))
    return feature
```

The implementation for the `compute_color_histograms`, `get_normals` and `compute_normal_histograms` functions can be checked on the [features.py](pr2_robot/scripts/features.py) file mainly using the exercises suggestion.

For debugging purposes the following line is added to output:

```python
rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
```

A segmentation and object recognition sample is as follows:

![Segmentation and object recognition][image6]

#### Find the Centroid

To calculate the centroid (average in x, y and z) of the set of points belonging to that each object a means operation is executed on top of the detected object cloud as a Numpy array:

```python
points_arr = ros_to_pcl(obj.cloud).to_array()
centroid = np.mean(points_arr, axis=0)[:3]
center_x = np.asscalar(centroid[0])
center_y = np.asscalar(centroid[1])
center_z = np.asscalar(centroid[2])
...
pick_pose.position.x = center_x
pick_pose.position.y = center_y
pick_pose.position.z = center_z
```

#### YAML files

To get the correct object group and place for each detected object, the following parameters are read and matched:

```python
object_list_param = rospy.get_param('/object_list')
dropbox_list_param = rospy.get_param('/dropbox')
```

Once the pipeline is complete, a yaml file with the scene detection (object name, pick_pose, etc.) is created using the provided `make_yaml_dict` function.

### Results

After running the pick_place_project launch file with the correct scene selected; run the [project_template.py](pr2_robot/scripts/project_template.py) file.

#### Scene 1

Execution:

```sh
roslaunch pr2_robot pick_place_project.launch
...
rosrun pr2_robot project_template.py
```

![scene 1][image3]

Output detail:

```
[INFO] [1542249282.783510, 2999.687000]: Detected 3 objects: ['biscuits', 'soap', 'soap2']
[INFO] [1542249292.556518, 3002.768000]: Detected 3 objects: ['biscuits', 'soap', 'soap2']
```

All the objects (3/3) are detected ([output_1.yaml](output_1.yaml)) for `test1.world`.

#### Scene 2

Execution:

```sh
roslaunch pr2_robot pick_place_project.launch scene_number:=2
...
rosrun pr2_robot project_template.py
```

![scene 2][image4]

Output detail:

```
[INFO] [1542249638.296998, 1120.688000]: Detected 5 objects: ['biscuits', 'book', 'soap', 'soap2', 'glue']
[INFO] [1542249648.505126, 1124.116000]: Detected 5 objects: ['biscuits', 'book', 'soap', 'soap2', 'glue']
```

All the objects (5/5) are detected ([output_2.yaml](output_1.yaml)) for `test2.world`.

#### Scene 3

Execution:

```sh
roslaunch pr2_robot pick_place_project.launch scene_number:=3
...
rosrun pr2_robot project_template.py
```

![scene 3][image5]

Output detail:

```
[INFO] [1542279724.850520, 1350.405000]: Detected 7 objects: ['snacks', 'biscuits', 'book', 'soap', 'eraser', 'soap2', 'sticky_notes']
[INFO] [1542279737.922417, 1354.255000]: Detected 7 objects: ['snacks', 'biscuits', 'book', 'soap', 'eraser', 'soap2', 'sticky_notes']
```

7/8 objects are detected ([output_3.yaml](output_1.yaml)) for `test3.world`.

### Discussion

One of the objects are not detected on the last scene because of occlusion from another object. Anyway when completing the `pr2_mover` routine, once the previous objects are correctly pick and placed, the object should be detected.

The basic pipeline as implemented.

The following TODO were not completed:

 * Rotate PR2 in place to capture side tables for the collision map.
 * Execute the pick_place routine.

### Resources

* [Project Baseline](https://github.com/ladrians/RoboND-Perception-Project-P3)
* [Original Repository](https://github.com/udacity/RoboND-Perception-Project)
* [Rubric](https://review.udacity.com/#!/rubrics/1067/view)

