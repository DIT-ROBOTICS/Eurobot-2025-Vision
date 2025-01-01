#include "pcl_preprocess.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

void PCLPreprocess::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud);
}

// KD-tree input: cloud1, cloud2
void PCLPreprocess::calculateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud1);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud1_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud1_normals);

    ne.setInputCloud(cloud2);
    pcl::PointCloud<pcl::Normal>::Ptr cloud2_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud2_normals);
}