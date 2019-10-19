Points Transform
================

pcl_apps::PointsTransformComponent has these topic interface.

+-------------------------+-------------------------------+---------------------+
| Input Topics            | Type                          | Description         |
+=========================+===============================+=====================+
| (param : ~/input_topic) | sensor_msgs::msg::PointCloud2 | Input Point Cloud   |
+-------------------------+-------------------------------+---------------------+

+------------------+-------------------------------+---------------------+
| Output Topics    | Type                          | Description         |
+==================+===============================+=====================+
| ~/output         | sensor_msgs::msg::PointCloud2 | Output Point Cloud  |
+------------------+-------------------------------+---------------------+

+--------------------+----------+---------------------------------------+---------+
| Parameter          | Type     | Description                           | Default |
+====================+==========+=======================================+---------+
| ~/output_frame_id  | String   | Frame ID of the Output Point Cloud    | ""      |
+--------------------+----------+---------------------------------------+---------+
| ~/input_topic      | String   | input topic name                      | ~/input |
+--------------------+----------+---------------------------------------+---------+

Description
----------------------------------------
pcl_apps::PointsTransformComponent subscribe PointCloud topic and transform it to the output_frame_id.

Requirements
----------------------------------------
1. frame_id of the input/output frame shoud be exist.

How to launch with single node
------------------------------

.. code-block:: bash

    ros2 run pcl_apps points_transform_node