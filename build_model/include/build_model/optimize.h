#ifndef BUNDLE_ADJUST_H
#define BUNDLE_ADJUST_H


#include "common_include.h"
#include "qrplane.h"
// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;
//pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//g2o
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/se3quat.h>

using namespace std;
using namespace cv;

namespace buildmodel
{
class Optimize
{
public:
    typedef std::shared_ptr<Optimize> Ptr;
    Optimize();

    //Member Variables
    vector< cv::Point3f > _CornersInCamCor;
    vector< vector< cv::Point2f > > _CornersInImage_list;
    vector< Mat > _R_list;
    vector< Mat > _t_list;
    Eigen::Matrix4d _result_pose;
    vector< Eigen::Matrix4d > _result_pose_list; // for multiPoseOptimize
    vector < pcl::PointCloud<pcl::PointXYZRGBA> > _cube_points_cloud_list;

    //Member Functions
    void mybundleAdjustment(
            const vector< Point3f > points_3d,
            const vector< Point2f > points_2d,
            const Mat& K,
            Mat& R, Mat& t );

    void bundleAdjustment (
        const vector< Point3f > points_3d,
        const vector< Point2f > points_2d,
        const Mat& K,
        Mat& R, Mat& t );

    void multiPoseOptimize (
        const vector< Point3f > points_3d,
        const vector< QRPlane > frames,
        const Mat& K);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
}

#endif // BUNDLE_ADJUST_H
