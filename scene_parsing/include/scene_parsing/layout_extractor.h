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
#include <pcl/surface/concave_hull.h>
#include <pcl/common/centroid.h>

namespace layout_extractor {

    struct ExtractorParams {
        double distance_threshold;
        double cluster_tolerance;
        int min_cluster_size;
        int max_cluster_size;
    };

    class LayoutExtractor {

        ExtractorParams params_;

    public:

        ~LayoutExtractor() = default;

        LayoutExtractor(layout_extractor::ExtractorParams &params) : params_(params) {}

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

    std::vector<double> linspace(double start, double stop, size_t resolution) {

        double delta = std::abs(stop - start) / (static_cast<double>(resolution) - 1);

        std::vector<double> points(resolution, 0);
        if (stop > start) {
            for (size_t i = 0; i < resolution; ++i) {
                points[i] = start + delta * i;
            }
        } else {
            for (size_t i = 0; i < resolution; ++i) {
                points[i] = start - delta * i;
            }
        }

        return points;
    }

    std::vector<double>
    line_intersection(std::vector<double> p1, std::vector<double> p2, std::vector<double> p3, std::vector<double> p4) {

        Eigen::Matrix2d px_top11, px_top12, px_top21, px_top22;
        px_top11 << p1[0], p1[1], p2[0], p2[1];
        px_top12 << p1[0], 1, p2[0], 1;
        px_top21 << p3[0], p3[1], p4[0], p4[1];
        px_top22 << p3[0], 1, p4[0], 1;

        Eigen::Matrix2d py_top11, py_top12, py_top21, py_top22;
        py_top11 << p1[0], p1[1], p2[0], p2[1];
        py_top12 << p1[1], 1, p2[1], 1;
        py_top21 << p3[0], p3[1], p4[0], p4[1];
        py_top22 << p3[1], 1, p4[1], 1;

        Eigen::Matrix2d bot11, bot12, bot21, bot22;
        bot11 << p1[0], 1, p2[0], 1;
        bot12 << p1[1], 1, p2[1], 1;
        bot21 << p3[0], 1, p4[0], 1;
        bot22 << p3[1], 1, p4[1], 1;

        Eigen::Matrix2d px_top, py_top, bot;
        px_top << px_top11.determinant(), px_top12.determinant(), px_top21.determinant(), px_top22.determinant();
        py_top << py_top11.determinant(), py_top12.determinant(), py_top21.determinant(), py_top22.determinant();
        bot << bot11.determinant(), bot12.determinant(), bot21.determinant(), bot22.determinant();

        double px = px_top.determinant() / bot.determinant();
        double py = py_top.determinant() / bot.determinant();

        std::vector<double> p = {px, py};

        return p;
    }

    std::vector<std::vector<double>> steiner_ellipse(double a, double b, size_t resolution) {

        std::vector<double> A, B;
        std::vector<double> A_space = linspace(a, -a, resolution + 2);
        A.assign(A_space.begin() + 1, A_space.end() - 1);
        std::vector<double> B_space = linspace(0, 2 * b, resolution + 2);
        B.assign(B_space.begin() + 1, B_space.end() - 1);

        std::vector<double> v1 = {a, 0};
        std::vector<double> v2 = {-a, 0};
        std::vector<double> p1 = {0, b};
        std::vector<double> p2 = {0, -b};

        std::vector<std::vector<double>> vertices;

        // quadrant 1
        vertices.push_back(v1);
        for (size_t i = 0; i < resolution; ++i) {
            std::vector<double> Ai = {A[i], 2 * b};
            std::vector<double> Bi = {a, B[i]};
            vertices.push_back(line_intersection(v1, Ai, v2, Bi));
        }

        // quadrant 2
        vertices.push_back(p1);
        for (size_t i = resolution; i > 0; --i) {
            std::vector<double> v_q1 = vertices[i];
            std::vector<double> v_q2 = {-v_q1[0], v_q1[1]};
            vertices.push_back(v_q2);
        }

        // quadrant 3
        vertices.push_back(v2);
        for (size_t i = 1; i <= resolution; ++i) {
            std::vector<double> v_q1 = vertices[i];
            std::vector<double> v_q3 = {-v_q1[0], -v_q1[1]};
            vertices.push_back(v_q3);
        }

        // quadrant 4
        vertices.push_back(p2);
        for (size_t i = resolution; i > 0; --i) {
            std::vector<double> v_q1 = vertices[i];
            std::vector<double> v_q2 = {v_q1[0], -v_q1[1]};
            vertices.push_back(v_q2);
        }

        return vertices;
    }

    std::vector<std::vector<double>> rectangle(double a, double b) {

        std::vector<double> v1 = {a, b};
        std::vector<double> v2 = {-a, b};
        std::vector<double> v3 = {-a, -b};
        std::vector<double> v4 = {a, -b};

        std::vector<std::vector<double>> vertices = {v1, v2, v3, v4};

        return vertices;

    }

    class LayoutSummarizer {

    public:

        ~LayoutSummarizer() = default;

        template<typename PointT>
        std::vector<std::vector<double>>
        concaveSummary(const pcl::PointCloud<PointT> &point_cloud, const double alpha) const {

            typename pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
            *cloud_ptr = point_cloud;

            typename pcl::PointCloud<PointT>::Ptr hull_cloud(new pcl::PointCloud<PointT>);
            pcl::ConcaveHull<PointT> concave_hull;
            concave_hull.setInputCloud(cloud_ptr);
            concave_hull.setAlpha(alpha);
            concave_hull.reconstruct(hull_cloud);

            std::vector<std::vector<double>> hull_vertices;
            for (auto p : hull_cloud.points) {
                std::vector<double> v = {p.x, p.y, p.z};
                hull_vertices.push_back(v);
            }

            return hull_vertices;
        }

        template<typename PointT>
        std::vector<std::vector<double>>
        rectangleSummary(const pcl::PointCloud<PointT> &point_cloud) const {

            Eigen::Vector4d centroid;
            Eigen::Matrix3d covariance;
            pcl::computeMeanAndCovarianceMatrix(point_cloud, covariance, centroid);

            Eigen::Matrix3d eigen_vectors;
            Eigen::Vector3d eigen_values;
            pcl::eigen33(covariance, eigen_vectors, eigen_values);

            // 2D rectangle points from eigenvalues
            double a = eigen_values(2);
            double b = eigen_values(1);
            std::vector<std::vector<double>> rectangle_2d = rectangle(a, b);

            // rotation
            Eigen::Matrix3d R;
            R.col(0) = eigen_vectors.col(2);
            R.col(1) = eigen_vectors.col(1);
            R.col(2) = eigen_vectors.col(2).cross(eigen_vectors.col(1));

            // transform
            std::vector<std::vector<double>> vertices = std::vector<std::vector<double>>(4, std::vector<double>(3, 0));
            for (size_t i = 0; i < 4; ++i) {

                double x = ((R(0, 0) * rectangle_2d[i][0]) + (R(0, 1) * rectangle_2d[i][1])) + centroid(0);
                double y = ((R(1, 0) * rectangle_2d[i][0]) + (R(1, 1) * rectangle_2d[i][1])) + centroid(1);
                double z = ((R(2, 0) * rectangle_2d[i][0]) + (R(2, 1) * rectangle_2d[i][1])) + centroid(2);

                vertices[i][0] = x;
                vertices[i][1] = y;
                vertices[i][2] = z;

            }

            return vertices;
        }

        template<typename PointT>
        std::vector<std::vector<double>>
        ellipsoidSummary(const pcl::PointCloud<PointT> &point_cloud) const {

            Eigen::Vector4d centroid;
            Eigen::Matrix3d covariance;
            pcl::computeMeanAndCovarianceMatrix(point_cloud, covariance, centroid);

            Eigen::Matrix3d eigen_vectors;
            Eigen::Vector3d eigen_values;
            pcl::eigen33(covariance, eigen_vectors, eigen_values);

            // 2D ellipse points from eigenvalues
            double a = eigen_values(2);
            double b = eigen_values(1);
            std::vector<std::vector<double>> ellipse_2d = steiner_ellipse(a, b, 3);

            // rotation
            Eigen::Matrix3d R;
            R.col(0) = eigen_vectors.col(2);
            R.col(1) = eigen_vectors.col(1);
            R.col(2) = eigen_vectors.col(2).cross(eigen_vectors.col(1));

            // transform
            std::vector<std::vector<double>> vertices = std::vector<std::vector<double>>(ellipse_2d.size(),
                                                                                         std::vector<double>(3, 0));
            for (size_t i = 0; i < ellipse_2d.size(); ++i) {

                double x = ((R(0, 0) * ellipse_2d[i][0]) + (R(0, 1) * ellipse_2d[i][1])) + centroid(0);
                double y = ((R(1, 0) * ellipse_2d[i][0]) + (R(1, 1) * ellipse_2d[i][1])) + centroid(1);
                double z = ((R(2, 0) * ellipse_2d[i][0]) + (R(2, 1) * ellipse_2d[i][1])) + centroid(2);

                vertices[i][0] = x;
                vertices[i][1] = y;
                vertices[i][2] = z;

            }

            return vertices;

        }

    };

}


#endif //SRC_LAYOUT_EXTRACTOR_H
