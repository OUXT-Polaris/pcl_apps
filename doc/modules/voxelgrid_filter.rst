Voexlgrid Filter
================

pcl_apps::VoexlGridFilterComponent has these topic interface.

+------------------+-------------------------------+---------------------+
| Input Topics     | Type                          | Description         |
+==================+===============================+=====================+
| ~/input          | sensor_msgs::msg::PointCloud2 | Input Point Cloud   |
+------------------+-------------------------------+---------------------+

+------------------+-------------------------------+---------------------+
| Output Topics    | Type                          | Description         |
+==================+===============================+=====================+
| ~/output         | sensor_msgs::msg::PointCloud2 | Output Point Cloud  |
+------------------+-------------------------------+---------------------+

+--------------------+----------+---------------------------------------+
| Parameter          | Type     | Description                           |
+====================+==========+=======================================+
| ~/leaf_size        | Double   | Leaf size of the Voexlgrid            |
+--------------------+----------+---------------------------------------+

Description
----------------------------------------
pcl_apps::VoexlGridFilterComponent subscribe PointCloud topic and publish downsampled pointcloud by using Voexlgrid Filter

Requirements
----------------------------------------
1. leaf_size should be over 0

How to launch with single node
------------------------------

.. code-block:: bash

    ros2 run pcl_apps voxelgrid_filter_node