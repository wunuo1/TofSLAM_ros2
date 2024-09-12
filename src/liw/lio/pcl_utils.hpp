#ifndef PCL_UTILS_HPP
#define PCL_UTILS_HPP

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree.h>

#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


namespace CT_ICP {

    /*
    * @brief Calculate the error between the two planes.
    */
    template <typename T>
    Eigen::Matrix<T, 3, 1> ominus(const Eigen::Matrix<T, 4, 1>& ground_plane_coeff, Eigen::Matrix<T, 4, 1> &local_ground_plane) {
        Eigen::Matrix<T, 3, 1> n_ground_plane_coeff = ground_plane_coeff.template segment<3>(0);
        Eigen::Matrix<T, 3, 1> n_local_ground_plane = local_ground_plane.template segment<3>(0);
        Eigen::Matrix<float, 3, 1> x(1, 0, 0);
        Eigen::Matrix<T, 3, 1> normal_x = x.template cast<T>();
        Eigen::Matrix<T, 3, 3> R = Eigen::Quaternion<T>::FromTwoVectors(n_local_ground_plane, normal_x).toRotationMatrix();
        Eigen::Matrix<T, 3, 1> n_ground_plane_coeff_after_rotation = R * n_ground_plane_coeff;
        Eigen::Matrix<T, 3, 1> result;
        result(0) = atan2(n_ground_plane_coeff_after_rotation(1), n_ground_plane_coeff_after_rotation(0));
        result(1) = atan2(n_ground_plane_coeff_after_rotation(2), n_ground_plane_coeff_after_rotation.template head<2>().norm());
        result(2) = -local_ground_plane(3) + ground_plane_coeff(3);
        return result;
    }

    // template <typename T>
    // Eigen::Matrix<T, 4, 1> ominus(const Eigen::Matrix<T, 4, 1>& ground_plane_coeff, Eigen::Matrix<T, 4, 1> &local_ground_plane) {
    //     Eigen::Matrix<T, 4, 1> n_ground_plane_coeff = ground_plane_coeff.template segment<3>(0);
    //     Eigen::Matrix<T, 4, 1> n_local_ground_plane = local_ground_plane.template segment<3>(0);
    //     Eigen::Matrix<T, 4, 1> result;
    //     result(0) = (n_ground_plane_coeff(0) - n_local_ground_plane(0)) * (n_ground_plane_coeff(0) - n_local_ground_plane(0));
    //     result(1) = (n_ground_plane_coeff(1) - n_local_ground_plane(1)) * (n_ground_plane_coeff(1) - n_local_ground_plane(1));
    //     result(2) = (n_ground_plane_coeff(2) - n_local_ground_plane(2)) * (n_ground_plane_coeff(2) - n_local_ground_plane(2));
    //     result(3) = (n_ground_plane_coeff(3) - n_local_ground_plane(3)) * (n_ground_plane_coeff(3) - n_local_ground_plane(3));
    //     return result;
    // }

    // template <typename T>
    // Eigen::Matrix<T, 3, 1> ominus(const Eigen::Matrix<T, 4, 1>& ground_plane_coeff, Eigen::Matrix<T, 4, 1> &local_ground_plane) {
    //     Eigen::Matrix<T, 4, 1> n_ground_plane_coeff = ground_plane_coeff.template segment<4>(0);
    //     Eigen::Matrix<T, 4, 1> n_local_ground_plane = local_ground_plane.template segment<4>(0);
    //     Eigen::Matrix<T, 3, 1> result;
    //     result(0) = (n_ground_plane_coeff(0)*n_ground_plane_coeff(3)  - n_local_ground_plane(0)*n_local_ground_plane(0)) * 
    //             (n_ground_plane_coeff(0)*n_ground_plane_coeff(3)  - n_local_ground_plane(0)*n_local_ground_plane(0));
    //     result(1) = (n_ground_plane_coeff(1)*n_ground_plane_coeff(3) - n_local_ground_plane(1)*n_local_ground_plane(3)) * 
    //              (n_ground_plane_coeff(1)*n_ground_plane_coeff(3) - n_local_ground_plane(1)*n_local_ground_plane(3));
    //     result(2) = (n_ground_plane_coeff(2)*n_ground_plane_coeff(3) - n_local_ground_plane(2)*n_local_ground_plane(3)) * 
    //              (n_ground_plane_coeff(2)*n_ground_plane_coeff(3) - n_local_ground_plane(2)*n_local_ground_plane(3));
    //     return result;
    // }


    /*
    * @brief Get the plane coeffs.
    */
    template <typename PointType>
    Eigen::VectorXf get_plane_coeffs (typename pcl::PointCloud<PointType>::Ptr lidar_cloud) {
        Eigen::VectorXf coeff(4);
        // typename pcl::PointCloud<PointType>::Ptr ground_cloud;
        // ground_cloud.reset(new pcl::PointCloud<PointType>);
        // for(const auto& p : lidar_cloud->points){
        //     // if(std::fabs(p.normal_y + 1.0) < 1e-5) {
        //     ground_cloud->push_back(p);
        //     // }
        // }
        // typename pcl::SampleConsensusModelPlane<PointType>::Ptr model(
        //     new pcl::SampleConsensusModelPlane<PointType>(ground_cloud));//定义待拟合平面的model，并使用待拟合点云初始化
        // pcl::RandomSampleConsensus<PointType> ransac(model);//定义RANSAC算法模型
        // ransac.setDistanceThreshold(0.2);//设定阈值
        // ransac.computeModel();//拟合
        // ransac.getModelCoefficients(coeff);//获取拟合平面参数，对于平面ax+by_cz_d=0，coeff分别按顺序保存a,b,c,d
        // make the normal upward
        // *********************************************restore************************************************
        // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // pcl::SACSegmentation<pcl::PointXYZI> seg;
        // seg.setOptimizeCoefficients(true);
        // seg.setModelType(pcl::SACMODEL_PLANE);
        // seg.setMethodType(pcl::SAC_RANSAC);
        // // seg.setMaxIterations(100);
        // seg.setDistanceThreshold(0.2);

        // seg.setInputCloud(lidar_cloud);

        // // seg.setIndices(inliers);
        // seg.segment(*inliers, *coefficients);
        // std::cout << "------------------------------------coefficients: " << coefficients->values[0] << " " << coefficients->values[1]
        //             << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

        // pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::ExtractIndices<pcl::PointXYZI> extract;
        // extract.setInputCloud(lidar_cloud);
        // extract.setIndices(inliers);
        // extract.setNegative(false);
        // extract.filter(*non_ground_cloud);

        // typename pcl::SampleConsensusModelPlane<PointType>::Ptr model(
        //     new pcl::SampleConsensusModelPlane<PointType>(non_ground_cloud));//定义待拟合平面的model，并使用待拟合点云初始化
        // pcl::RandomSampleConsensus<PointType> ransac(model);//定义RANSAC算法模型
        // ransac.setDistanceThreshold(0.2);//设定阈值
        // ransac.computeModel();//拟合
        // ransac.getModelCoefficients(coeff);

        // coeff[0] = coefficients->values[0];
        // coeff[1] = coefficients->values[1];
        // coeff[2] = coefficients->values[2];
        // coeff[3] = coefficients->values[3];
        // // 法向量颠倒个方向
        // if(coeff.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
        //     coeff *= -1.0f;
        // }
        // ****************************************************************************************************
        coeff[0] = 0.0;
        coeff[1] = 0.0;
        coeff[2] = 1.0;
        coeff[3] = 0.0;
        return coeff;
    }
}


#endif // PCL_UTILS_HPP