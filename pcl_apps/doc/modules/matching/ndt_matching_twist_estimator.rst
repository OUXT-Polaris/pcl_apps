NDT Matching Twist Estimator
================

pcl_apps::NdtMatchingTwistEstimatorComponent has these topic interface.

+-----------------------------------+---------------------------------+----------------------------------+
| Input Topics                      | Type                            | Description                      |
+===================================+=================================+==================================+
| (param : ~/input_cloud_topic)     | sensor_msgs::msg::PointCloud2   | Input point cloud                |
+-----------------------------------+---------------------------------+----------------------------------+

+-------------------------+----------------------------------+----------------------------+
| Output Topics           | Type                             | Description                |
+=========================+==================================+============================+
| ~/current_twist         | geometry_msgs::msg::TwistStamped | Current tiwst of the robot |
+-------------------------+----------------------------------+----------------------------+

+-----------------------------+----------+---------------------------------------+----------------+---------+
| Parameter          　　     | Type     | Description                           | Default        | Dynamic |
+=============================+==========+=======================================+================+=========+
| ~/input_frame_id            | String   | Frame ID of the input cloud           | map            | False   |
+-----------------------------+----------+---------------------------------------+----------------+---------+
| ~/input_cloud_topic         | String   | Topic name of the input cloud         | ~/input        | False   |
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
pcl_apps::NdtMatchingTwistEstimatorComponent subscribes input point cloud and estimate current twist

Requirements
----------------------------------------
1. transform_epsilon,step_size,resolution must be over 0
2. max_iterations must be over 1

How to launch with single node
------------------------------

.. code-block:: bash

    ros2 run  pcl_apps ndt_matching_node