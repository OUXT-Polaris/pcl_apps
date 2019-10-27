NDT Matching
================

pcl_apps::NdtMatchingComponent has these topic interface.

+-----------------------------------+---------------------------------+----------------------------------+
| Input Topics                      | Type                            | Description                      |
+===================================+=================================+==================================+
| (param : ~/input_cloud_topic)     | sensor_msgs::msg::PointCloud2   | Input point cloud                |
+-----------------------------------+---------------------------------+----------------------------------+
| (param : ~/reference_cloud_topic) | sensor_msgs::msg::PointCloud2   | Reference point cloud            |
+-----------------------------------+---------------------------------+----------------------------------+
| (param : ~/initial_pose_topic)    | geometry_msgs::msg::PoseStamped | Initial pose of the ndt matching |
+-----------------------------------+---------------------------------+----------------------------------+

+-------------------------+---------------------------------+------------------------------------------------------------------+
| Output Topics           | Type                            | Description                                                      |
+=========================+=================================+==================================================================+
| ~/current_relative_pose | geometry_msgs::msg::PoseStamped | Relative pose of the input_cloud in (param:~/reference_frame_id) |
+-------------------------+---------------------------------+------------------------------------------------------------------+

+-----------------------------+----------+---------------------------------------+----------------+---------+
| Parameter          　　     | Type     | Description                           | Default        | Dynamic |
+=============================+==========+=======================================+================+=========+
| ~/reference_frame_id        | Int      | Frame ID of the reference input cloud | map            | False   |
+-----------------------------+----------+---------------------------------------+----------------+---------+
| ~/reference_cloud_topic     | String   | Topic name of the reference cloud     | ~/reference    | False   |
+-----------------------------+----------+---------------------------------------+----------------+---------+
| ~/input_cloud_topic         | String   | Topic name of the input cloud         | ~/input        | False   |
+-----------------------------+----------+---------------------------------------+----------------+---------+
| ~/initial_pose_topic        | String   | Initial pose topic                    | ~/initial_pose | False   |
+-----------------------------+----------+---------------------------------------+----------------+---------+
| ~/transform_epsilon         | Double   | Transform epsilon of the ndt matching | 1.0            | True    |
+-----------------------------+----------+---------------------------------------+----------------+---------+
| ~/step_size                 | Double   | Stap size of the ndt matching         | 0.1            | True    |
+-----------------------------+----------+---------------------------------------+----------------+---------+
| ~/resolution                | Double   | Resolution of the ndt                 | 1.0            | True    |
+-----------------------------+----------+---------------------------------------+----------------+---------+
| ~/max_iterations            | Int      | Maximum nunber of iterations          | 35             | True    |
+-----------------------------+----------+---------------------------------------+----------------+---------+

Description
----------------------------------------
pcl_apps::NdtMatchingComponent subscribes input/reference point cloud and estimate relative pose between two point clouds.

Requirements
----------------------------------------
1. frame_id of the reference_cloud must be same
2. transform_epsilon,step_size,resolution must be over 0
3. max_iterations must be over 1

How to launch with single node
------------------------------

.. code-block:: bash

    ros2 run  pcl_apps ndt_matching_node