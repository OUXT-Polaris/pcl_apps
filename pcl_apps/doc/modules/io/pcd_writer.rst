Pcd Writer
================

pcl_apps::PcdWriterComponent subscribe pointcloud topics and save this data as pcd format when the user calls write_pcd service.

+-------------------------+-------------------------------+---------------------+
| Input Topics            | Type                          | Description         |
+=========================+===============================+=====================+
| (param : ~/input_topic) | sensor_msgs::msg::PointCloud2 | Input Point Cloud   |
+-------------------------+-------------------------------+---------------------+

+--------------------+----------+---------------------------------------+---------+---------+
| Parameter          | Type     | Description                           | Default | Dynamic |
+====================+==========+=======================================+=========+=========+
| ~/input_topic      | String   | input topic name                      | ~/input | False   |
+--------------------+----------+---------------------------------------+---------+---------+

+-------------------------+-------------------------------+-----------------------------------+
| Service                 | Type                          | Description                       |
+=========================+===============================+===================================+
| ~write_pcd              | pcl_apps_msgs::msg::WritePcd  | Service for writing point cloud   |
+-------------------------+-------------------------------+-----------------------------------+

.. code-block:: bash

    ros2 run pcl_apps pcd_writer_node