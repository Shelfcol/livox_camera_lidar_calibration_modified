//*****************************************************************************************************
//    获得单帧点云中的角反射器角点
//
//    按中shift键,然后在角点处鼠标左击
//    source devel/setup.bash && rosrun cam_laser_calib pcd_points_get 111.pcd  
//    pcl_viewer 111.pcd
//**************************************************************************************************

#include <ros/ros.h>
#include <string>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "common.h"
 
using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
 
string pcd_folder;
string output_path;
vector<PointT>  corners;
void getParameters() {
    cout << "Get the parameters from the launch file" << endl;
    if (!ros::param::get("pcd_folder", pcd_folder)) {
        cout << "Can not get the value of pcd_folder" << endl;
        exit(1);
    }
    if (!ros::param::get("output_path", output_path)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
}

void writeData(const string filename, const float x, const float y,float z, uint mode) {
    ofstream outfile;
    static bool write_first = true;
    if(write_first){
        write_first = false;
        outfile = ofstream(filename.c_str(), ios_base::trunc); // 读取并清空文件夹
    } else
    {
        outfile = ofstream(filename.c_str(), ios_base::app); // 追加模式
    }
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    switch(mode) {
        case(0):
            outfile << "lidar" << endl;
            outfile << "1" << endl;
            break;
        case(1):
            outfile << "2" << endl;
            break;
        case(2):
            outfile << "3" << endl;
            break;
        case(3):
            outfile << "4" << endl;
            break;
        default:
            cout << "[writeData] - Code error, unknown mode" << endl;
            exit(0);
    }
    outfile << float2str(x) << "        " << float2str(y) <<"        "<<float2str(z)<< endl;
}

// 用于将参数传递给回调函数的结构体
struct CallbackArgs {
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


void pickPointCallback(const pcl::visualization::PointPickingEvent &event, void *args) {
    CallbackArgs *data = (CallbackArgs *) args;
    if (event.getPointIndex() == -1){
        printf("click no data\n");
        return;
    }
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    if(abs(current_point.x)<1e-5 && abs(current_point.y)<1e-5 && abs(current_point.z)<1e-5) {return;}
    data->clicked_points_3d->points.push_back(current_point);
    // 绘制红色点
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points"); // 移除上次的点云
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points"); // 画这次的点
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;

    // 因为有时候点击一次会多次显示这个点，所以这里在加入之前进行判断
    if(!corners.empty()&& abs(corners.back().x-current_point.x)<1e-5&&abs(corners.back().y-current_point.y)<1e-5 && abs(corners.back().z-current_point.z)<1e-5)
    {
        return;
    }
    corners.push_back(current_point);
}
 
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cornerPcd");
    getParameters();
    vector<string> filenames;
    GetFileNames(pcd_folder, filenames);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    size_t idx = 0;
    for(; idx<filenames.size();)
    {
        string file_name = filenames[idx];
        corners.clear();
        std::cout<<"打开pcd点云文件: "<<file_name<<'\n'; 

        // 加载点云
        if (pcl::io::loadPCDFile(file_name, *cloud) == -1) {
            std::cerr << "could not load file: " << file_name << std::endl;
        }
        std::cout << cloud->points.size() << std::endl;
        // 显示点云
        viewer->addPointCloud(cloud, "cloud");
        viewer->setCameraPosition(0, 0, -2, 1, 0, 0, 0);
        // 添加点拾取回调函数
        CallbackArgs  cb_args;
        PointCloudT::Ptr clicked_points_3d(new PointCloudT);
        cb_args.clicked_points_3d = clicked_points_3d;
        cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
        viewer->registerPointPickingCallback(pickPointCallback, (void*)&cb_args);
        std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
    
        viewer->spin();
        std::cout << "done." << std::endl;
    
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
        }
        viewer->removeAllPointClouds(); // 清空点云
        printf("writing data\n");

        if(corners.size()!=4) { // 当点数不对的时候，就重新对这张点云图进行操作
            continue;
        }

        for(uint t = 0; t < corners.size(); ++t) {
            printf("(%.3f %.3f %.3f)\n", (float)corners[t].x, corners[t].y,corners[t].z);
            writeData(output_path, corners[t].x, corners[t].y,corners[t].z, t);
        }
        ++idx;
    }
    return 0;
}