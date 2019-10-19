Points Concatenate
==================

pcl_apps::PointsConcatenateComponent has these topic interface.

+------------------+-------------------------------+---------------------+
| Input Topics     | Type                          | Description         |
+==================+===============================+=====================+
| ~/input0         | sensor_msgs::msg::PointCloud2 | Input Point Cloud0  |
+------------------+-------------------------------+---------------------+
| ~/input1         | sensor_msgs::msg::PointCloud2 | Input Point Cloud1  |
+------------------+-------------------------------+---------------------+
| ~/input2         | sensor_msgs::msg::PointCloud2 | Input Point Cloud2  |
+------------------+-------------------------------+---------------------+
| ~/input3         | sensor_msgs::msg::PointCloud2 | Input Point Cloud3  |
+------------------+-------------------------------+---------------------+
| ~/input4         | sensor_msgs::msg::PointCloud2 | Input Point Cloud4  |
+------------------+-------------------------------+---------------------+
| ~/input5         | sensor_msgs::msg::PointCloud2 | Input Point Cloud5  |
+------------------+-------------------------------+---------------------+
| ~/input6         | sensor_msgs::msg::PointCloud2 | Input Point Cloud6  |
+------------------+-------------------------------+---------------------+
| ~/input7         | sensor_msgs::msg::PointCloud2 | Input Point Cloud7  |
+------------------+-------------------------------+---------------------+

+------------------+-------------------------------+---------------------+
| Output Topics    | Type                          | Description         |
+==================+===============================+=====================+
| ~/output         | sensor_msgs::msg::PointCloud2 | Output Point Cloud  |
+------------------+-------------------------------+---------------------+

+--------------------+----------+---------------------------------------+
| Parameter          | Type     | Description                           |
+====================+==========+=======================================+
| ~/num_input        | Int      | Number of Input Point Cloud Topics    |
+--------------------+----------+---------------------------------------+

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