#pragma once

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLHeader.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/shared_ptr.hpp>

namespace boost
{

    namespace serialization
    {

        template<class Archive>
        void serialize(Archive & ar, pcl::PCLHeader & g, const unsigned int version)
        {
            (void)version;
            ar & g.seq;
            ar & g.stamp;
            ar & g.frame_id;
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointXYZ & g, const unsigned int version)
        {
            (void)version;
            ar & g.x;
            ar & g.y;
            ar & g.z;
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointXYZRGB & g, const unsigned int version)
        {
            (void)version;
            ar & g.x;
            ar & g.y;
            ar & g.z;
            ar & g.rgba;
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointNormal & g, const unsigned int version)
        {
            (void)version;
            ar & g.x;
            ar & g.y;
            ar & g.z;
            ar & g.normal[0];
            ar & g.normal[1];
            ar & g.normal[2];
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::Normal & g, const unsigned int version)
        {
            (void)version;
            ar & g.normal[0];
            ar & g.normal[1];
            ar & g.normal[2];
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointXYZI& g, const unsigned int version)
        {
            (void)version;
            ar & g.x;
            ar & g.y;
            ar & g.z;
            ar & g.intensity;
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointXYZINormal & g, const unsigned int version)
        {
            (void)version;
            ar & g.x;
            ar & g.y;
            ar & g.z;
            ar & g.intensity;
            ar & g.curvature;
            ar & g.normal_x;
            ar & g.normal_y;
            ar & g.normal_z;
        }


        template<class Archive>
        void serialize(Archive & ar,Eigen::Quaternion<float> & g, const unsigned int version)
        {
            (void)version;
            ar & g.w();
            ar & g.x();
            ar & g.y();
            ar & g.z();
        }

        template<class Archive>
        void serialize(Archive & ar, Eigen::Translation<float, 3> & g, const unsigned int version)
        {
            (void)version;
            ar & g.x();
            ar & g.y();
            ar & g.z();
        }

        template<class Archive>
        void serialize(Archive & ar, Eigen::UniformScaling<float> & g, const unsigned int version)
        {
            (void)version;
            ar & g.factor();
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointIndices & g, const unsigned int version)
        {
            (void)version;
            ar & g.indices;
            ar & g.header;
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointCloud<pcl::PointNormal> & g, const unsigned int version)
        {
            (void)version;
            ar & g.header;
            ar & g.points;
            ar & g.height;
            ar & g.width;
            ar & g.is_dense;
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZ> & g, const unsigned int version)
        {
            (void)version;
            ar & g.header;
            ar & g.points;
            ar & g.height;
            ar & g.width;
            ar & g.is_dense;
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZRGB> & g, const unsigned int version)
        {
            (void)version;
            ar & g.header;
            ar & g.points;
            ar & g.height;
            ar & g.width;
            ar & g.is_dense;
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointCloud<pcl::Normal> & g, const unsigned int version)
        {
            (void)version;
            ar & g.header;
            ar & g.points;
            ar & g.height;
            ar & g.width;
            ar & g.is_dense;
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZI> & g, const unsigned int version)
        {
            (void)version;
            ar & g.header;
            ar & g.points;
            ar & g.height;
            ar & g.width;
            ar & g.is_dense;
        }


        template<class Archive>
        void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZINormal> & g, const unsigned int version)
        {
            (void)version;
            ar & g.header;
            ar & g.points;
            ar & g.height;
            ar & g.width;
            ar & g.is_dense;
        }

        template<class Archive>
        void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZINormal>::Ptr & g, const unsigned int version)
        {
            // (void)version;
            ar & g->header;
            ar & g->points;
            ar & g->height;
            ar & g->width;
            ar & g->is_dense;
        }

        template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
        inline void serialize(Archive & ar,
                Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & matrix,
                const unsigned int /* aVersion */)
        {
            Eigen::Index rows = matrix.rows();
            Eigen::Index cols = matrix.cols();
            ar & (rows);
            ar & (cols);
            if(rows != matrix.rows() || cols != matrix.cols())
                matrix.resize(rows, cols);
            if(matrix.size() !=0)
                ar &  boost::serialization::make_array(matrix.data(), rows * cols);
        }


        /* serialization for CV Mat */
        template<class Archive>
        void serialize(Archive &ar, cv::Mat& mat, const unsigned int)
        {
            int cols, rows, type;
            bool continuous;

            if (Archive::is_saving::value) {
                cols = mat.cols; rows = mat.rows; type = mat.type();
                continuous = mat.isContinuous();
            }

            ar & cols & rows & type & continuous;

            if (Archive::is_loading::value)
                mat.create(rows, cols, type);

            if (continuous) {
                const unsigned int data_size = rows * cols * mat.elemSize();
                ar & boost::serialization::make_array(mat.ptr(), data_size);
            }
            else {
                const unsigned int row_size = cols*mat.elemSize();
                for (int i = 0; i < rows; i++) {
                    ar & boost::serialization::make_array(mat.ptr(i), row_size);
                }
            }

        }


        // /* serialization for CV Mat */
        // template<class Archive>
        // void save(Archive &ar, const ::cv::Mat &m, const unsigned int file_version)
        // {
        //     cv::Mat m_ = m;
        //     if (!m.isContinuous())
        //         m_ = m.clone();
        //     size_t elem_size = m_.elemSize();
        //     size_t elem_type = m_.type();
        //     ar & m_.cols;
        //     ar & m_.rows;
        //     ar & elem_size;
        //     ar & elem_type;

        //     const size_t data_size = m_.cols * m_.rows * elem_size;

        //     ar & boost::serialization::make_array(m_.ptr(), data_size);
        // }

        // template<class Archive>
        // void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
        // {
        //     int cols, rows;
        //     size_t elem_size, elem_type;

        //     ar & cols;
        //     ar & rows;
        //     ar & elem_size;
        //     ar & elem_type;

        //     m.create(rows, cols, elem_type);
        //     size_t data_size = m.cols * m.rows * elem_size;

        //     ar & boost::serialization::make_array(m.ptr(), data_size);
        // }





    // template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    // inline void serialize( Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,const unsigned int file_version)
    // {
    //     (void) file_version;
    //     int rows = t.rows(), cols = t.cols();
    //     ar & rows;
    //     ar & cols;
    //     if( rows * cols != t.size() )
    //     t.resize( rows, cols );

    //     for(int i=0; i<t.size(); i++)
    //     ar & t.data()[i];
    // }


    }
}

