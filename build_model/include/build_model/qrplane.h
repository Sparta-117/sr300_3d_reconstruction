#ifndef QRPLANE_H
#define QRPLANE_H

#include "common_include.h"
#include "camera.h"
//pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
// #include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
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

using namespace std;
using namespace cv;

namespace buildmodel
{
class QRPlane
{
public:
    typedef std::shared_ptr<QRPlane> Ptr;
    QRPlane();
    QRPlane(int numberOfQROnSide,cv::Matx33f camera_intrinsic_matrix,cv::Vec4f distParam);

    class QR
    {
    public:
        float QRNumber;//the number of QR markers in a plane
        float QR_TAG_SIZE; //meter
        float length1;
        float length2;
        float length3;
        float length4;
        float length5;
        float cubeSide;
        float cubeHeight;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        vector<Eigen::Vector3f> basePositions;
        vector<Eigen::Vector4f> cubeVertexInWorld;

    } _qr;


    class SingleQRCorner
    {
    public:
        typedef std::shared_ptr<SingleQRCorner> Ptr;
        int id;
        Eigen::Vector4f CornerInWorldCor[4];
        Eigen::Vector4f CornerInCameraCor[4];
        Eigen::Vector2f CornerInImageCor[4];
        float CornerIndepthImage[4];
    };

    vector<SingleQRCorner> _QRCornersList;
    vector<SingleQRCorner> _AllQRCornersList;


    //Member Variables
    int _numberOfQROnSide;
    Mat _rgb_image;
    Mat _depth_image;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cube_cloud;
    vector< int > _ids;
    vector< vector< Point2f > > _corners;
    vector< vector< Point2f > > _rejected;
    std::vector< cv::Point3f > _AllQRCornersInWorldCor;
    std::vector< cv::Point3f > _AllQRCornersInCamCor;
    std::vector<cv::Point3f> _CornersInWorldCor;
    std::vector<cv::Point3f> _CornersInCameraCor;
    std::vector<cv::Point2f> _CornersInImageCor;
    cv::Matx33f _camera_intrinsic_matrix;
    float _fx,_fy,_cx,_cy;
    cv::Vec4f _distParam;
    cv::Mat _rvec;
    cv::Mat _tvec;
    Eigen::Matrix3f _rotation;
    Eigen::Vector3f _translation;
    Eigen::Matrix4f _transform_world2camera;
    Eigen::Matrix4f _transform_camera2world;
    cv::Point2f _cubeVertexInImage[8];

    //Member functions
    void detectQRCode();
    bool whetherDectectQRCode();
    void GetTransfromCameraToWorld();
    void DrawAxis(Mat &tmp);
    void DrawCube(Mat &tmp);
    void CutCubeInDepthImage(Mat &result_depth_image);
    void CutCubeInDepthImageAndPointsCloud(Mat &result_depth_image,
                                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cube_points_cloud);
    void TransformPointsCloudCorToWorld(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cube_points_cloud);
//    void CutCubeInPointsCloud();
    void AllQRCornersInCamCor();
    void SaveQRPlaneData(int index);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


private:
    class TEN_x_TEN : public QR
    {
    public:
        TEN_x_TEN()
        {
            QRNumber    = __QRNumber;
            QR_TAG_SIZE = __QR_TAG_SIZE;
            length1     = __length1;
            length2     = __length2;
            length3     = __length3;
            length4     = __length4;
            length5     = __length5;
            cubeSide    = __cubeSide;
            cubeHeight  = __cubeHeight;
            dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

            for(int i = 0; i < 100; i++) {
                basePositions.push_back(__basePositions[i]);
            }

            for(int j = 0; j < 8; j++) {
                cubeVertexInWorld.push_back(__cubeVertexInWorld[j]);
            }
        }
    private:
        float __QRNumber = 64;
        float __QR_TAG_SIZE = 0.053; //meter
        float __length1 = 0.26775;
        float __length2 = 0.20825;
        float __length3 = 0.14875;
        float __length4 = 0.08925;
        float __length5 = 0.02975;
        float __cubeSide = 0.36;
        float __cubeHeight = 0.36;

        Eigen::Vector3f __basePositions[100] = {{-__length1, __length1, 0}, //0~9
                                                {-__length2, __length1, 0},
                                                {-__length3, __length1, 0},
                                                {-__length4, __length1, 0},
                                                {-__length5, __length1, 0},
                                                {__length5, __length1, 0},
                                                {__length4, __length1, 0},
                                                {__length3, __length1, 0},
                                                {__length2, __length1, 0},
                                                {__length1, __length1, 0},
                                                {-__length1, __length2, 0},//10~19
                                                {-__length2, __length2, 0},
                                                {-__length3, __length2, 0},
                                                {-__length4, __length2, 0},
                                                {-__length5, __length2, 0},
                                                {__length5, __length2, 0},
                                                {__length4, __length2, 0},
                                                {__length3, __length2, 0},
                                                {__length2, __length2, 0},
                                                {__length1, __length2, 0},
                                                {-__length1, __length3, 0},//20~21
                                                {-__length2, __length3, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {__length2, __length3, 0},//28~29
                                                {__length1, __length3, 0},
                                                {-__length1, __length4, 0},//30~31
                                                {-__length2, __length4, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {__length2, __length4, 0},//38~39
                                                {__length1, __length4, 0},
                                                {-__length1, __length5, 0},//40~41
                                                {-__length2, __length5, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {__length2, __length5, 0},//48~49
                                                {__length1, __length5, 0},
                                                {-__length1, -__length5, 0},//50~51
                                                {-__length2, -__length5, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {__length2, -__length5, 0},//58~59
                                                {__length1, -__length5, 0},
                                                {-__length1, -__length4, 0},//60~61
                                                {-__length2, -__length4, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {__length2, -__length4, 0},//68~69
                                                {__length1, -__length4, 0},
                                                {-__length1, -__length3, 0},//70~71
                                                {-__length2, -__length3, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {0, 0, 0},
                                                {__length2, -__length3, 0},//78~79
                                                {__length1, -__length3, 0},
                                                {-__length1, -__length2, 0},//80~89
                                                {-__length2, -__length2, 0},
                                                {-__length3, -__length2, 0},
                                                {-__length4, -__length2, 0},
                                                {-__length5, -__length2, 0},
                                                {__length5, -__length2, 0},
                                                {__length4, -__length2, 0},
                                                {__length3, -__length2, 0},
                                                {__length2, -__length2, 0},
                                                {__length1, -__length2, 0},
                                                {-__length1, -__length1, 0},//90~99
                                                {-__length2, -__length1, 0},
                                                {-__length3, -__length1, 0},
                                                {-__length4, -__length1, 0},
                                                {-__length5, -__length1, 0},
                                                {__length5, -__length1, 0},
                                                {__length4, -__length1, 0},
                                                {__length3, -__length1, 0},
                                                {__length2, -__length1, 0},
                                                {__length1, -__length1, 0}
                                               };

        Eigen::Vector4f __cubeVertexInWorld[8] = {{-__cubeSide/2, __cubeSide/2, 0, 1},
                                                  {__cubeSide/2, __cubeSide/2, 0, 1},
                                                  {__cubeSide/2, -__cubeSide/2, 0, 1},
                                                  {-__cubeSide/2, -__cubeSide/2, 0, 1},
                                                  {-__cubeSide/2, __cubeSide/2, __cubeSide, 1},
                                                  {__cubeSide/2, __cubeSide/2, __cubeSide, 1},
                                                  {__cubeSide/2, -__cubeSide/2, __cubeSide, 1},
                                                  {-__cubeSide/2, -__cubeSide/2, __cubeSide, 1}
                                                 };
    };

    class SIX_x_SIX : public QR
    {
    public:
        SIX_x_SIX()
        {
            QRNumber    = __QRNumber;
            QR_TAG_SIZE = __QR_TAG_SIZE;
            length1     = __length1;
            length2     = __length2;
            length3     = __length3;
            length4     = 0;
            length5     = 0;
            cubeSide    = __cubeSide;
            cubeHeight  = __cubeHeight;
            dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

            for(int i = 0; i < 20; i++) {
                basePositions.push_back(__basePositions[i]);
            }

            for(int j = 0; j < 8; j++) {
                cubeVertexInWorld.push_back(__cubeVertexInWorld[j]);
            }
        }

    private:
        float __QRNumber = 20;
        float __QR_TAG_SIZE = 0.035;  //meter
        float __length1 = 0.1075;
        float __length2 = 0.0645;
        float __length3 = 0.0215;
        float __cubeSide = 0.26;
        float __cubeHeight = 0.30;


        Eigen::Vector3f __basePositions[20] = {{-__length1, __length1, 0}, //0
                                               {-__length2, __length1, 0},//1
                                               {-__length3, __length1, 0},//2
                                               {__length3, __length1, 0},//3
                                               {__length2, __length1, 0},//4
                                               {__length1, __length1, 0},//5
                                               {-__length1, __length2, 0},//6
                                               {__length1, __length2, 0},//7
                                               {-__length1, __length3, 0},//8
                                               {__length1, __length3, 0},//9
                                               {-__length1, -__length3, 0},//10
                                               {__length1, -__length3, 0},//11
                                               {-__length1, -__length2, 0},//12
                                               {__length1, -__length2, 0},//13
                                               {-__length1, -__length1, 0},//14
                                               {-__length2, -__length1, 0},//15
                                               {-__length3, -__length1, 0},//16
                                               {__length3, -__length1, 0},//17
                                               {__length2, -__length1, 0},//18
                                               {__length1, -__length1, 0}//19
                                              };

        Eigen::Vector4f __cubeVertexInWorld[8] = {{-__cubeSide/2, __cubeSide/2, 0, 1},
                                                  {__cubeSide/2, __cubeSide/2, 0, 1},
                                                  {__cubeSide/2, -__cubeSide/2, 0, 1},
                                                  {-__cubeSide/2, -__cubeSide/2, 0, 1},
                                                  {-__cubeSide/2, __cubeSide/2, __cubeHeight, 1},
                                                  {__cubeSide/2, __cubeSide/2, __cubeHeight, 1},
                                                  {__cubeSide/2, -__cubeSide/2, __cubeHeight, 1},
                                                  {-__cubeSide/2, -__cubeSide/2, __cubeHeight, 1}
                                                 };
    };
};
}

#endif // QRPLANE_H
