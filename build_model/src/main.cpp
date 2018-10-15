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
    //pcl::visualization::PCLVisualizer viewer1 ("cube_points_cloud_from_camera");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    //config file of camera
    std::string config_filename = "/home/mzm/sr300_3d_reconstruction/src/build_model/config/default2.yaml";
    buildmodel::Camera::Ptr camera1 (new buildmodel::Camera);
    camera1->_camera_config_filename = config_filename;

    buildmodel::QRPlane::Ptr plane_pre;

    //ros node
    ros::init(argc, argv, "build_model");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<realsense_msgs::realsense_msgs>("get_image");
    realsense_msgs::realsense_msgs srv;
    srv.request.start = true;
    sensor_msgs::Image msg_rgb;
    sensor_msgs::Image msg_depth;
    ros::Rate loop_rate(200);

    while(ros::ok())
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
        Eigen::Matrix4f current_pose;
        Eigen::Matrix4f pose_to_reference;
        Eigen::Matrix4d pose_to_reference_optimized;
        vector< int > id_list_cur;
        vector< int > id_list_match;
        vector< cv::Point3f > CornersInCamCor_Match;
        vector< cv::Point2f > CornersInImage_Match;
        vector< cv::Point2f > CornersInImage_Pre;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cube_points_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cube_points_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        buildmodel::Optimize::Ptr opt1(new buildmodel::Optimize());
        buildmodel::QRPlane::Ptr plane1(new buildmodel::QRPlane(numberOfQRCodeOnSide,
                                                                  camera1->_camera_intrinsic_matrix,
                                                                  camera1->_distParam));
        plane1->_rgb_image = rgb_image;
        plane1->_depth_image = depth_image;
        plane1->detectQRCode();
        if(plane1->whetherDectectQRCode())
        {
            plane1->GetTransfromCameraToWorld();
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

        if(index == 1)
        {
            plane_pre = plane1;
            reference_pose = plane1->_transform_world2camera;
            plane_pre->AllQRCornersInCamCor();
//            opt1->_cube_points_cloud_list.push_back(*cube_points_cloud);
//            for(int i=0;i<plane_pre->_AllQRCornersInCamCor.size();i++)
//            {

//            }
        }
        else
        {
//            buildmodel::Optimize::Ptr opt1(new buildmodel::Optimize());
            for(int i=0;i<plane1->_QRCornersList.size();i++)
            {
                for(int j=0;j<plane_pre->_AllQRCornersList.size();j++)
                {
                    if(plane_pre->_AllQRCornersList[j].id == plane1->_QRCornersList[i].id)
                    {
                        id_list_match.push_back(plane1->_QRCornersList[i].id);
                        cout<<"id:"<<plane1->_QRCornersList[i].id<<endl;
                        for(int m=0;m<4;m++)
                        {
                            Point3f SingleCornerInCamCor_pre;
                            Point2f SingleCornerInImage_cur;
//                            Point2f SingleCornerInImage_pre;
//                            Eigen::Vector3f test;
                            SingleCornerInCamCor_pre = Point3f(plane_pre->_AllQRCornersList[j].CornerInCameraCor[m][0],
                                                    plane_pre->_AllQRCornersList[j].CornerInCameraCor[m][1],
                                                    plane_pre->_AllQRCornersList[j].CornerInCameraCor[m][2]);
//                            Eigen::Vector4f XYZ;
//                            XYZ <<  SingleCornerInCamCor_pre.x,
//                                    SingleCornerInCamCor_pre.y,
//                                    SingleCornerInCamCor_pre.z,
//                                    1;
//                            test = camera1->_intrinsic_matrix*XYZ;


                            SingleCornerInImage_cur = Point2f(plane1->_QRCornersList[i].CornerInImageCor[m][0],
                                                   plane1->_QRCornersList[i].CornerInImageCor[m][1]);
//                            SingleCornerInImage_pre = Point2f(plane_pre->_AllQRCornersList[i].CornerInImageCor[m][0],
//                                                    plane_pre->_AllQRCornersList[i].CornerInImageCor[m][1]);
                            CornersInCamCor_Match.push_back(SingleCornerInCamCor_pre);
//                            CornersInImage_Pre.push_back(SingleCornerInImage_pre);
                            CornersInImage_Match.push_back(SingleCornerInImage_cur);
//                            CornersInImage_Match_Pre.push_back(Point2f(test(0)/test(2),test(1)/test(2)));

//                            cout<<"test:"<<test(0)/test(2)<<" "<<test(1)/test(2)<<endl;
                            cout<<"CornerInImage:"<<SingleCornerInImage_cur<<endl;
                            cout<<"CornerInCamCor_pre:"<<SingleCornerInCamCor_pre<<endl;
                            cout<<"Real depth:"<<plane1->_QRCornersList[i].CornerIndepthImage[m]<<endl;
                        }
                    }
                }
            }
            for(int i=0;i<plane1->_QRCornersList.size();i++)
                id_list_cur.push_back(plane1->_QRCornersList[i].id);
            for(int j=0;j<plane_pre->_AllQRCornersList.size();j++)
                id_list_pre.push_back(plane_pre->_AllQRCornersList[j].id);

            current_pose = plane1->_transform_world2camera;
            pose_to_reference = reference_pose*current_pose.inverse();
//            pose_to_preframe = (pre_pose*current_pose.inverse()).inverse();
//            pre_pose = current_pose;
            cout<<"T from QR:"<<endl;
            cout<<pose_to_reference<<endl;
//            cout<<"T between frames:"<<endl;
//            cout<<pose_to_preframe<<endl;

//            Eigen::Matrix4f tmp_test;
//            tmp_test = pose_to_reference.inverse();

            Mat K = ( Mat_<double> ( 3,3 ) << 615, 0, 314, 0, 615, 238, 0, 0, 1 );
            Mat R = (Mat_<double> ( 3,3 )<< pose_to_reference(0,0),pose_to_reference(0,1),pose_to_reference(0,2),
                                                pose_to_reference(1,0),pose_to_reference(1,1),pose_to_reference(1,2),
                                                pose_to_reference(2,0),pose_to_reference(2,1),pose_to_reference(2,2));
            Mat t = ( Mat_<double> ( 3,1 ) <<pose_to_reference(0,3),pose_to_reference(1,3),pose_to_reference(2,3));

//            opt1->_cube_points_cloud_list.push_back(cube_points_cloud);
//            opt1->_CornersInCamCor.push_back();

            Mat r2, t2;
            solvePnP ( CornersInCamCor_Match, CornersInImage_Match, K, Mat(), r2, t2, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
            Mat R2;
            cv::Rodrigues ( r2, R2 ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

            cout<<"R2="<<endl<<R2.inv()<<endl;
            cout<<"t2="<<endl<<-1*R2.inv()*t2<<endl;

//            R= R.inv();
//            t= -1*R*t;
//            Mat R1 = Mat_<double> ( 3,3 );
//            Mat t1 = Mat_<double> ( 3,1 );
//            Mat R;
//            cv::Rodrigues(plane1->_rvec,R);
//            R = R.inv();
//            Mat t;
//            t = -1*R*plane1->_tvec;

            opt1->bundleAdjustment(CornersInCamCor_Match,
                                   CornersInImage_Match,
                                   K,
                                   R2,
                                   t2);

            pose_to_reference_optimized = opt1->_result_pose.inverse();


            cout<<"pre_id:";
            for(int a=0;a<id_list_pre.size();a++)
            {
                cout<<id_list_pre[a]<<" ";
            }
            cout<<endl;
            cout<<"cur_id:";
            for(int a=0;a<id_list_cur.size();a++)
            {
                cout<<id_list_cur[a]<<" ";
            }
            cout<<endl;
            cout<<"match_id:";
            for(int a=0;a<id_list_match.size();a++)
            {
                cout<<id_list_match[a]<<" ";
            }
            cout<<endl;

            for(int i=0;i<CornersInImage_Match.size();i++)
            {
                circle(rgb_tmp,CornersInImage_Match[i],2,Scalar(0,0,255),2);
//                circle(rgb_tmp,CornersInImage_Pre[i],2,Scalar(0,255,0),2);
            }

            id_list_pre.clear();
            id_list_cur.clear();
            id_list_match.clear();

//            plane_pre = plane1;
            CornersInCamCor_Match.clear();
            CornersInImage_Match.clear();
//            CornersInImage_Pre.clear();

        }


//        imshow("rgb",rgb_image);
//        imshow("depth",depth_tmp);
        imshow("result",rgb_tmp);
        imshow("depth_result",result_depth_image);
        waitKey(1);

        if((index <30)&&(index>1))
        {
//        pcl::transformPointCloud (*cube_points_cloud, *transformed_cube_points_cloud,pose_to_reference_optimized);
        pcl::transformPointCloud (*cube_points_cloud, *transformed_cube_points_cloud,pose_to_reference);
        *final_cloud = *final_cloud + *transformed_cube_points_cloud;
        }
        if(index == 30)
        {
            pcl::PLYWriter writer;
            writer.write("/home/mzm/new_sr300_build_model/src/model.ply",*final_cloud);
        }
    }

}
