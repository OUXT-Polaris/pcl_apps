Radius Outlier Removal
======================

pcl_apps::RadiusOutlierRemovalComponent has these topic interface.

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

+----------------------------------+----------+-------------------------------------------+---------+---------+
| Parameter                        | Type     | Description                               | Default | Dynamic |
+==================================+==========+===========================================+=========+=========+
| ~/search_radius                  | Double   | Radius of the search sphere               | 1.0     | True    |
+----------------------------------+----------+-------------------------------------------+---------+---------+
| ~/min_neighbors_in_search_radius | Int      | Min numbers of neighbors in search radius | 1       | True    |
+----------------------------------+----------+-------------------------------------------+---------+---------+
| ~/input_topic                    | String   | input topic name                          | ~/input | False   |
+----------------------------------+----------+-------------------------------------------+---------+---------+

Description
----------------------------------------
pcl_apps::RadiusOutlierRemovalComponent subscribe PointCloud topic and publish filterd topic 

Requirements
----------------------------------------
1. min_neighbors_in_search_radius should be over 1
2. search_radius shoud be over 0
