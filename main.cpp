
#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


// // 用户接口
string pathKitti = "/home/zlp/catkin_ws/Dataset/KITTI/sequences/00/velodyne/";
string pathPoseFile = "/home/zlp/catkin_ws/Dataset/KITTI/sequences/data_odometry_poses/dataset/poses/00.txt";


// // 全局变量
int countScan = 0;
vector<vector<float>> poseVector;
void readCloud(string pathBinFile,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
    Matrix4f TransForm;
    TransForm<<poseVector[countScan][0],poseVector[countScan][1],poseVector[countScan][2],poseVector[countScan][3],
            poseVector[countScan][4],poseVector[countScan][5],poseVector[countScan][6],poseVector[countScan][7],
            poseVector[countScan][8],poseVector[countScan][9],poseVector[countScan][10],poseVector[countScan][11],
            0,0,0,1;

    Matrix4f tr;
    tr<<4.276802385584e-04, -9.999672484946e-01 ,-8.084491683471e-03 ,-1.198459927713e-02 ,
            -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01 ,-5.403984729748e-02 ,
            9.999738645903e-01 ,4.859485810390e-04, -7.206933692422e-03 ,-2.921968648686e-01,
            0,0,0,1;

    FILE *lidar;
    lidar = fopen(pathBinFile.c_str(),"rb");
    if(lidar == NULL){
        cout<<"BinFile open fail"<<endl;
        return ;
    }
    float *data = (float*)malloc(1000000*sizeof(float));
    int32_t Num = fread(data,sizeof(float),1000000,lidar)/4;
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;
    for (int32_t i=0; i<Num; i++) {
        PointXYZRGBA p;
        Vector4f Plidar,Pworld;
        Plidar<<*px,*py,*pz,1;
        Pworld=TransForm*tr*Plidar;
        // Pworld=TransForm*Plidar;
        p.x=Pworld(0);
        p.y=Pworld(1);
        p.z=Pworld(2);
        if(p.y >= 1.8){
            p.b = 255;
            p.g = 0;
        }
        if(p.y <= 0){
            p.b = 0;
            p.g = 255;
        }
        if(p.y<1.8 && p.y>0){
            p.b = (int)((p.y-0)/1.8*255);
            p.g = 255-p.b;
        }
        cloud->points.push_back(p);
        px+=4; py+=4; pz+=4; pr+=4;
    }
    fclose(lidar);
    cloud->width=1;
    cloud->height=static_cast<uint32_t>(cloud->size());
}
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,void *viewer_void){
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
    if(event.keyDown()){
        string KeyFromBoard = event.getKeySym();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSingleScan(new pcl::PointCloud<pcl::PointXYZRGBA>);

        if(KeyFromBoard == "Right"){
            cout<<"putin:next"<<endl;
            countScan ++ ;
        }
        if(KeyFromBoard == "Left"){
            cout<<"putin:last"<<endl;
            countScan --;
        }
        if(KeyFromBoard == "Up"){
            cout<<"putin:last 10"<<endl;
            countScan -= 10;
        }
        if(KeyFromBoard == "Down"){
            cout<<"putin:next 10"<<endl;
            countScan += 10;
        }
        cout<<KeyFromBoard<<endl;
        string pathBinFile = pathKitti+to_string(1000000+countScan).substr(1,6)+".bin";
        cout<<"now show "<<countScan<<endl<<endl;
        readCloud(pathBinFile,cloudSingleScan);
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGBA> fildColor(cloudSingleScan, "y");
        viewer->updatePointCloud(cloudSingleScan,fildColor, "sample cloud");
    }
}
void readPose(string pathPoseFile,vector<vector<float>> &poseVector){
    ifstream poseFile(pathPoseFile);
    if ( !poseFile){
        cout<<"pose file open failed"<<endl;
        return ;
    }
    while (!poseFile.eof()){
        vector<float> poseTemp;
        for (int i=0;i<12;i++){
            float temp;
            poseFile>>temp;
            poseTemp.push_back(temp);
        }
        poseVector.push_back(poseTemp);
    }
    poseFile.close();
    return ;
}
int main()
{
    readPose(pathPoseFile,poseVector);
    cout<<"get Pose:"<<poseVector.size()<<endl;
    cout<<"funciton:"<<endl;
    cout<<"Left:last cloud"<<endl;
    cout<<"Up:last 10th cloud"<<endl;
    cout<<"Right:next cloud"<<endl;
    cout<<"Down:next 10th cloud"<<endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOri(new pcl::PointCloud<pcl::PointXYZRGBA>);
    string pathBinFile = pathKitti+to_string(1000000+countScan).substr(1,6)+".bin";
    readCloud(pathBinFile,cloudOri);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void *)viewer.get());
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGBA>(cloudOri, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    int i = 1;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    return 0;
}
