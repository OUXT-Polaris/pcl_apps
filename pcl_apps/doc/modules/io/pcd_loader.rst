Pcd Loader
================

Pcd Writer Component load pcd file and publish as sensor_msgs::msg::PointCloud2

+-------------------------+-------------------------------+---------------------+
| Output Topics           | Type                          | Description         |
+=========================+===============================+=====================+
| (param : ~/out_topic)   | sensor_msgs::msg::PointCloud2 | Output Point Cloud  |
+-------------------------+-------------------------------+---------------------+

+--------------------+----------+---------------------------------------+----------+---------+
| Parameter          | Type     | Description                           | Default  | Dynamic |
+====================+==========+=======================================+==========+=========+
| ~/output_topic     | String   | output topic name                     | ~/output | False   |
+--------------------+----------+---------------------------------------+----------+---------+
| ~/file_path        | String   | pcd file path                         |    ""    | False   |
+--------------------+----------+---------------------------------------+----------+---------+
| ~/inference_id     | String   | FrameID of the cloud from PCD file    |   map    | False   |
+--------------------+----------+---------------------------------------+----------+---------+