#include "optimize.h"

using namespace std;
using namespace cv;

namespace buildmodel
{
Optimize::Optimize()
{}

void Optimize::mybundleAdjustment(
        const vector< Point3f > points_3d,
        const vector< Point2f > points_2d,
        const Mat& K,
        Mat& R, Mat& t )
{
    // 先构造求解器
    g2o::SparseOptimizer    optimizer;
    // 使用Cholmod中的线性方程求解器
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new
            g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();
//    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new
//            g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();

    //6*3 parameter
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3(linearSolver);
    //L-M algorithm
    g2o::OptimizationAlgorithmLevenberg* algorithm = new
            g2o::OptimizationAlgorithmLevenberg(block_solver);

    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(false);

    //pose vertex
    Eigen::Matrix3d R_mat;
    R_mat <<   R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    //add two vertex for pose(pre and cur)
    for(int i=0;i<2;i++)
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if(i==0)
        {
            v->setFixed(true);//first pose is set to 0
            v->setEstimate(g2o::SE3Quat());
        }
        else
        {
            v->setEstimate ( g2o::SE3Quat (R_mat,Eigen::Vector3d ( t.at<double> ( 0,0 ),
                                                                   t.at<double> ( 1,0 ),
                                                                   t.at<double> ( 2,0 ) )) );

        }
        optimizer.addVertex(v);
    }

    //3d corner vertex
    //the first vertex
    for(size_t i=0;i<points_3d.size();i++)
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId(2+i);
        v->setMarginalized(true);
        v->setEstimate ( Eigen::Vector3d ( points_3d[i].x,points_3d[i].y,points_3d[i].z ) );
        optimizer.addVertex(v);
    }

    //prepare camera parameter
    g2o::CameraParameters* camera = new
            g2o::CameraParameters(K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    //edges
    vector<g2o::EdgeProjectXYZ2UV*> edges;
    for(size_t i=0;i<points_2d.size();i++)
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0,dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2)));
        edge->setVertex(1,dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(1)));
        edge->setMeasurement(Eigen::Vector2d(points_2d[i].x,points_2d[i].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0, 0);
        //kernal function
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    cout<<"开始优化"<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cout<<"优化完毕"<<endl;

    //我们比较关心两帧之间的变换矩阵
    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d pose = v->estimate();
    cout<<"Pose="<<endl<<pose.matrix()<<endl;
//    cout<<"test Pose="<<pose.inverse().matrix()<<endl;

}


void Optimize::bundleAdjustment (
    const vector< Point3f > points_3d,
    const vector< Point2f > points_2d,
    const Mat& K,
    Mat& R, Mat& t )
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<   R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        ) );
    optimizer.addVertex ( pose );

    int index = 1;
    for ( const Point3f p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges
    index = 1;
    for ( const Point2f p:points_2d )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose  );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    Eigen::Isometry3d result_pose = pose->estimate();
    cout<<"T="<<endl<<result_pose.matrix() <<endl;

    _result_pose = result_pose.matrix();
//    cout<<"test T="<<endl<<result_pose.inverse().matrix()<<endl;
}

void Optimize::multiPoseOptimize(const vector< Point3f > points_3d,
                                 const vector< QRPlane > frames,
                                 const Mat& K)
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    cout<<"Set pose vertex."<<endl;
    int index = 0;
    for( QRPlane f1:frames)
    {
        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
        Eigen::Matrix3d R_mat;
        Eigen::Matrix4d p1 = f1._transform_world2camera.cast<double>();
        R_mat <<   p1(0,0), p1(0,1), p1(0,2),
                   p1(1,0), p1(1,1), p1(1,2),
                   p1(2,0), p1(2,1), p1(2,2);
        cout<<"No."<<index<<"pose = "<<endl<<p1<<endl;
        pose->setId ( index++ );
        pose->setEstimate ( g2o::SE3Quat (
                                R_mat,
                                Eigen::Vector3d ( p1(0,3), p1(1,3), p1(2,3))
                            ) );
        optimizer.addVertex ( pose );

    }

    cout<<"Set 3d point vertex."<<endl;
    //vertex
    for ( const Point3f p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        // cout<<"No."<<index<<"point:"<<p.x<<" "<<p.y<<" "<<p.z<<endl;
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        // point->setFixed(true);
        optimizer.addVertex ( point );
    }

    cout<<"Set camera intrinsics."<<endl;
    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    cout<<"Set edges."<<endl;
    // edges
    index = frames.size();
    int edge_index = 1;
    int frames_index = 0;
    for ( const QRPlane f1:frames )
    {
        for( QRPlane::SingleQRCorner p:f1._QRCornersList)
        {
            for(int i=0;i<4;i++)
            {
                // cout<<"Edge No."<<edge_index<<" point No."<<index + p.id*4 +i<<" Pose No."<<frames_index<<endl;
                g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
                edge->setId ( edge_index );
                edge->setVertex (0,dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(index + p.id*4 +i)));
                edge->setVertex(1,dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(frames_index)));
                Eigen::Vector2d edge_measurement = p.CornerInImageCor[i].cast<double>();
                edge->setMeasurement ( edge_measurement );
                edge->setParameterId ( 0,0 );
                edge->setInformation ( Eigen::Matrix2d::Identity() );
                optimizer.addEdge ( edge );
                edge_index++;
            }
        }
        frames_index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 6 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;

    //我们比较关心两帧之间的变换矩阵
    for(int i=0;i<frames.size();i++)
    {
        g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(i) );
        Eigen::Isometry3d result_pose = v->estimate();
        cout<<"No."<<i<<"Pose="<<endl<<result_pose.matrix()<<endl;
        _result_pose_list.push_back(result_pose.matrix());
    }
}
}
