Points Concatenate
==================

pcl_apps::PointsConcatenateComponent has these topic interface.

+--------------------------+-------------------------------+---------------------+
| Input Topics             | Type                          | Description         |
+==========================+===============================+=====================+
| (param : ~/input_topic0) | sensor_msgs::msg::PointCloud2 | Input Point Cloud0  |
+--------------------------+-------------------------------+---------------------+
| (param : ~/input_topic1) | sensor_msgs::msg::PointCloud2 | Input Point Cloud1  |
+--------------------------+-------------------------------+---------------------+
| (param : ~/input_topic2) | sensor_msgs::msg::PointCloud2 | Input Point Cloud2  |
+--------------------------+-------------------------------+---------------------+
| (param : ~/input_topic3) | sensor_msgs::msg::PointCloud2 | Input Point Cloud3  |
+--------------------------+-------------------------------+---------------------+
| (param : ~/input_topic4) | sensor_msgs::msg::PointCloud2 | Input Point Cloud4  |
+--------------------------+-------------------------------+---------------------+
| (param : ~/input_topic5) | sensor_msgs::msg::PointCloud2 | Input Point Cloud5  |
+--------------------------+-------------------------------+---------------------+
| (param : ~/input_topic6) | sensor_msgs::msg::PointCloud2 | Input Point Cloud6  |
+--------------------------+-------------------------------+---------------------+
| (param : ~/input_topic7) | sensor_msgs::msg::PointCloud2 | Input Point Cloud7  |
+--------------------------+-------------------------------+---------------------+

+------------------+-------------------------------+---------------------+
| Output Topics    | Type                          | Description         |
+==================+===============================+=====================+
| ~/output         | sensor_msgs::msg::PointCloud2 | Output Point Cloud  |
+------------------+-------------------------------+---------------------+

+-------------------+----------+---------------------------------------+----------+
| Parameter         | Type     | Description                           | Default  |
+===================+==========+=======================================+==========+
| ~/num_input       | Int      | Number of Input Point Cloud Topics    | 2        |
+-------------------+----------+---------------------------------------+----------+
| ~/input_topic0    | String   | input topic name                      | ~/input0 |
+-------------------+----------+---------------------------------------+----------+
| ~/input_topic1    | String   | input topic name                      | ~/input1 |
+-------------------+----------+---------------------------------------+----------+
| ~/input_topic2    | String   | input topic name                      | ~/input2 |
+-------------------+----------+---------------------------------------+----------+
| ~/input_topic3    | String   | input topic name                      | ~/input3 |
+-------------------+----------+---------------------------------------+----------+
| ~/input_topic4    | String   | input topic name                      | ~/input4 |
+-------------------+----------+---------------------------------------+----------+
| ~/input_topic5    | String   | input topic name                      | ~/input5 |
+-------------------+----------+---------------------------------------+----------+
| ~/input_topic6    | String   | input topic name                      | ~/input6 |
+-------------------+----------+---------------------------------------+----------+
| ~/input_topic7    | String   | input topic name                      | ~/input7 |
+-------------------+----------+---------------------------------------+----------+


Description
----------------------------------------
pcl_apps::PointsConcatenateComponent subscribes multiple pointcloud topics and concatenate them.
pcl_apps::PointsConcatenateComponent use approximate time synchronizer in ROS2.

Requirements
----------------------------------------
1. all of input topic frame should be same.  
2. all of the input topic should be published if you want to publish output pointcloud. 
    

How to launch with single node
------------------------------

.. code-block:: bash

    ros2 run  pcl_apps points_concatenate_node