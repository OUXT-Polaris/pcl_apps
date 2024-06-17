#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

using namespace pcl;
using namespace std;

int main() {
    PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>());
    PointCloud<PointXYZI>::Ptr cloud_filtered(new PointCloud<PointXYZI>());

    if (io::loadPCDFile<PointXYZI>("input.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file input.pcd\n");
        return -1;
    }

    cout << "Loaded " << cloud->points.size() << "data points from input.pcd with the following fields: x y z intensity\n";

    PassThrough<PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud_filtered);

    cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << endl;

    io::savePCDFileASCII("output.pcd", *cloud_filtered);

    return 0;
}