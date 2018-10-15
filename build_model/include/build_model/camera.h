#ifndef CAMERA_H
#define CAMERA_H

#include "common_include.h"
#include "config.h"
// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;
//pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
// #include <pcl/console/print.h>
// #include <pcl/console/parse.h>
// #include <pcl/console/time.h>

// #include <pcl/filters/voxel_grid.h>//滤波
// #include <pcl/filters/filter.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/registration/icp.h> //ICP(iterative closest point)配准

// #include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
// #include <pcl/registration/correspondence_estimation.h>
// #include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
// #include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
// #include <pcl/features/fpfh.h>
// #include <pcl/registration/ia_ransac.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree_flann.h>

using namespace cv;
using namespace std;

namespace buildmodel
{
// Pinhole RGBD camera model
class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr;
    float   _fx, _fy, _cx, _cy, _depth_scale;
    float   _dispara1, _dispara2, _dispara3, _dispara4;

    Camera();
    Camera ( float fx, float fy, float cx, float cy, float depth_scale=0 ) :
        _fx ( fx ), _fy ( fy ), _cx ( cx ), _cy ( cy ), _depth_scale ( depth_scale )
    {
        _intrinsic_matrix << _fx,0,_cx,0,
                            0,_fy,_cy,0,
                            0,0,1,0;
        _camera_intrinsic_matrix << _fx, 0, _cx,
                                    0, _fy, _cy,
                                    0, 0, 1
                                    ;
        _distParam << 0,0,0,0;
    }


    //Member Variables
    std::string _camera_config_filename;
    Eigen::Matrix<float,3,4> _intrinsic_matrix;
    cv::Matx33f _camera_intrinsic_matrix;
    cv::Vec4f _distParam;

    // coordinate transform: world, camera, pixel
    Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
    Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
    Vector2d camera2pixel( const Vector3d& p_c );
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1 );
    Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
#endif // CAMERA_H
