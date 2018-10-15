#include "common_include.h"
#include "config.h"
#include "camera.h"
#include "qrplane.h"
#include "optimize.h"
#include "realsense_msgs/realsense_msgs.h"

using namespace std;
using namespace cv;
using namespace buildmodel;

#define numberOfQRCodeOnSide 6

int main(int argc,char *argv[])
{
    Mat rgb_image;
    Mat rgb_tmp;
    Mat depth_image;
    Mat depth_tmp;
    int index = 0;
    vector<int> id_list_pre;
    Eigen::Matrix4f reference_pose;
    Eigen::Matrix4f world_pose;
    //pcl::visualization::PCLVisualizer viewer1 ("cube_points_cloud_from_camera");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr optimized_final_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    //config file of camera
    std::string config_filename = "/home/mzm/sr300_3d_reconstruction/src/build_model/config/default2.yaml";
    buildmodel::Camera::Ptr camera1 (new buildmodel::Camera);
    camera1->_camera_config_filename = config_filename;

    buildmodel::Optimize::Ptr opt1(new buildmodel::Optimize());
    buildmodel::QRPlane::Ptr plane_pre;
    vector< cv::Point3f > AllQRCornersInWorldCor;
    vector< pcl::PointCloud<pcl::PointXYZRGBA> > point_cloud_list;
    vector< QRPlane > plane_list;
    // Mat K = ( Mat_<double> ( 3,3 ) << 620.845, 0, 311.832, 0, 620.845, 241.665, 0, 0, 1 );
    Mat K = ( Mat_<double> ( 3,3 ) << 615, 0, 314, 0, 615, 238, 0, 0, 1 );

    //ros node
    ros::init(argc, argv, "build_model");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<realsense_msgs::realsense_msgs>("get_image");
    realsense_msgs::realsense_msgs srv;
    srv.request.start = true;
    sensor_msgs::Image msg_rgb;
    sensor_msgs::Image msg_depth;
    ros::Rate loop_rate(200);

    while((ros::ok())&&(index<3))
    {
        if (client.call(srv))
        {
            try
            {
              msg_rgb = srv.response.rgb_image;
              msg_depth = srv.response.depth_image;
              cout<<"running!"<<endl;
              rgb_image = cv_bridge::toCvCopy(msg_rgb)->image;
              depth_image = cv_bridge::toCvCopy(msg_depth)->image;
              normalize(depth_image,depth_tmp,255,0,NORM_MINMAX);
              depth_tmp.convertTo(depth_tmp, CV_8UC1, 1.0);
            }
            catch (cv_bridge::Exception& e)
            {
              ROS_ERROR("cv_bridge exception: %s", e.what());
              return 1;
            }
            depth_image.convertTo(depth_image,CV_32F);
        }

        if( !rgb_image.data )
        {
            printf( " No image data \n " );
            return -1;
        }
        rgb_image.copyTo(rgb_tmp);
        Mat result_depth_image = Mat::zeros(depth_image.size(),CV_32FC1);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cube_points_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cube_points_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        buildmodel::QRPlane::Ptr plane1(new buildmodel::QRPlane(numberOfQRCodeOnSide,
                                                                  camera1->_camera_intrinsic_matrix,
                                                                  camera1->_distParam));
        plane1->_rgb_image = rgb_image;
        plane1->_depth_image = depth_image;
        plane1->detectQRCode();
        if(plane1->whetherDectectQRCode())
        {
            plane1->GetTransfromCameraToWorld();
            plane1->AllQRCornersInCamCor();
            AllQRCornersInWorldCor = plane1->_AllQRCornersInWorldCor;
//            plane1->DrawAxis(rgb_tmp);
//            plane1->DrawCube(rgb_tmp);
            plane1->CutCubeInDepthImageAndPointsCloud(result_depth_image,cube_points_cloud);
            index++;
        }
        else
        {
            cout<<"Not find QR code!"<<endl;
            continue;
        }

        plane_list.push_back(*plane1);
        point_cloud_list.push_back(*cube_points_cloud);
        cout<<"plane_list size:"<<plane_list.size()<<endl;
        reference_pose = plane1->_transform_world2camera;
        world_pose =  plane1->_transform_camera2world;
        pcl::transformPointCloud (*cube_points_cloud, *transformed_cube_points_cloud,world_pose);
        *final_cloud = *final_cloud + *transformed_cube_points_cloud;
    }
    opt1->multiPoseOptimize(AllQRCornersInWorldCor,plane_list,K);

    cout<<"Get optimized point cloud."<<endl;
    for(int i=0;i<plane_list.size();i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cube_points_cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
        Eigen::Matrix4d world_pose2 = (opt1->_result_pose_list[i]).inverse();
        pcl::transformPointCloud (point_cloud_list[i], *transformed_cube_points_cloud2,world_pose2);
        *optimized_final_cloud = *optimized_final_cloud + *transformed_cube_points_cloud2;
        cout<<"No."<<i<<"point cloud optimized."<<endl;
    }
    pcl::PLYWriter writer;
    writer.write("/home/mzm/sr300_3d_reconstruction/src/model.ply",*final_cloud);
    writer.write("/home/mzm/sr300_3d_reconstruction/src/optimized_model.ply",*optimized_final_cloud);

}
