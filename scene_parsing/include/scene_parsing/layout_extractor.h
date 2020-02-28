//
// Created by armon on 2/26/20.
//

#ifndef SRC_LAYOUT_EXTRACTOR_H
#define SRC_LAYOUT_EXTRACTOR_H

#include <eigen3/Eigen/Core>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>

namespace layout_extractor {

    struct Params {
        double distance_threshold;
        double cluster_tolerance;
        int min_cluster_size;
        int max_cluster_size;
    };

    class LayoutExtractor {

        Params params_;

    public:

        ~LayoutExtractor() = default;

        LayoutExtractor(layout_extractor::Params &params) : params_(params) {}

        template<typename PointT>
        std::vector<pcl::PointCloud<PointT>>
        extractSegmentsAtPlane(const pcl::PointCloud<PointT> &point_cloud, const std::vector<double> &plane) const {

            typename pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
            *cloud_ptr = point_cloud;

            Eigen::VectorXf plane_coefficients(4);
            plane_coefficients << plane[0], plane[1], plane[2], plane[3];

            // layout cloud
            typename pcl::PointCloud<PointT>::Ptr layout_cloud(new pcl::PointCloud<PointT>);
            std::vector<int> inliers;
            pcl::SampleConsensusModelPlane<PointT> sac_model_plane(cloud_ptr);
            sac_model_plane.selectWithinDistance(plane_coefficients, params_.distance_threshold, inliers);
            typename pcl::PointCloud<PointT>::Ptr projected_cloud(new pcl::PointCloud<PointT>);
            sac_model_plane.projectPoints(inliers, plane_coefficients, *projected_cloud);
            pcl::copyPointCloud(*projected_cloud, inliers, *layout_cloud);

            // cluster segments
            std::vector<pcl::PointIndices> cluster_indices;
            typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
            tree->setInputCloud(layout_cloud);
            pcl::EuclideanClusterExtraction<PointT> extraction;
            extraction.setClusterTolerance(params_.cluster_tolerance);
            extraction.setMinClusterSize(params_.min_cluster_size);
            if (params_.max_cluster_size > 0) { extraction.setMaxClusterSize(params_.max_cluster_size); }
            extraction.setSearchMethod(tree);
            extraction.setInputCloud(layout_cloud);
            extraction.extract(cluster_indices);

            // save
            std::vector<pcl::PointCloud<PointT>> layout_clusters;
            for (const auto &index_set : cluster_indices) {

                pcl::PointCloud<PointT> cluster;
                for (auto i : index_set.indices) {
                    cluster.points.push_back(layout_cloud->points[i]);
                }
                layout_clusters.push_back(cluster);

            }

            return layout_clusters;
        }

        template<typename PointT>
        pcl::PointCloud<PointT>
        filterPointCloud(const pcl::PointCloud<PointT> &point_cloud, const double leaf_size) const {

            typename pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
            *cloud_ptr = point_cloud;

            pcl::VoxelGrid<PointT> voxel_grid;
            pcl::PointCloud<PointT> filtered_cloud;
            voxel_grid.setInputCloud(cloud_ptr);
            voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxel_grid.filter(filtered_cloud);

            return filtered_cloud;
        }

    };

}


#endif //SRC_LAYOUT_EXTRACTOR_H
