#include <ros/ros.h>
#include <iostream>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <string>
#include <opencv2/core/eigen.hpp>
#include "common.h"
#include "result_verify.h"

using namespace std;

typedef pcl::PointXYZRGB PointType;
Eigen::Matrix3d inner;
string lidar_path, photo_path, intrinsic_path, extrinsic_path;
int error_threshold;
vector<float> init;


class external_cali {
public:
    external_cali(PnPData p) {
        pd = p;
    }

    template <typename T>
    bool operator()(const T *_q, const T *_t, T *residuals) const {
        Eigen::Matrix<T, 3, 3> innerT = inner.cast<T>();
        Eigen::Quaternion<T> q_incre{_q[ 3 ], _q[ 0 ], _q[ 1 ], _q[ 2 ]};
        Eigen::Matrix<T, 3, 1> t_incre{_t[ 0 ], _t[ 1 ], _t[ 2 ]};

        Eigen::Matrix<T, 3, 1> p_l(T(pd.x), T(pd.y), T(pd.z));
        Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
        Eigen::Matrix<T, 3, 1> p_2 = innerT * p_c;

        residuals[0] = p_2[0]/p_2[2] - T(pd.u);
        residuals[1] = p_2[1]/p_2[2] - T(pd.v);

        return true;
    }

    static ceres::CostFunction *Create(PnPData p) {
        return (new ceres::AutoDiffCostFunction<external_cali, 2, 4, 3>(new external_cali(p)));
    }

private:
    PnPData pd;
};

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_lidar_path", lidar_path)) {
        cout << "Can not get the value of input_lidar_path" << endl;
        exit(1);
    }
    if (!ros::param::get("input_photo_path", photo_path)) {
        cout << "Can not get the value of input_photo_path" << endl;
        exit(1);
    }
    if (!ros::param::get("intrinsic_path", intrinsic_path)) {
        cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("extrinsic_path", extrinsic_path)) {
        cout << "Can not get the value of extrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("error_threshold", error_threshold)) {
        cout << "Can not get the value of error_threshold" << endl;
        exit(1);
    }
    init.resize(12);
    if (!ros::param::get("init_value", init)) {
        cout << "Can not get the value of init_value" << endl;
        exit(1);
    }
}

void PnpTrans(const vector<PnPData>& pData, const Eigen::Matrix3d& inner,Eigen::Matrix3d& Rot, Eigen::Vector3d& trans)
{
    vector<cv::Point3d> pts_3d;
    vector<cv::Point2d> pts_2d;
    for_each(pData.begin(),pData.end(),[&](auto a){
        pts_3d.emplace_back(a.x,a.y,a.z);
        pts_2d.emplace_back(a.u,a.v);
    });
    cv::Mat K(2, 3, CV_64F);
    cv::eigen2cv(inner,K);
    cv::Mat r,t;
    cv::solvePnP(pts_3d,pts_2d,K,cv::Mat(),r,t);
    cv::Mat R;
    cv::Rodrigues(r,R);

    cv::cv2eigen(R,Rot);
    cv::cv2eigen(t,trans);
    // trans<<t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0);

    std::cout<<"R = "<<R<<std::endl;
    std::cout<<"t = "<<t<<std::endl;
    std::cout<<"R = "<<Rot<<std::endl;
    std::cout<<"t = "<<trans<<std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "getExt_modified");
    getParameters();
    vector<PnPData> pData;
    getData(lidar_path, photo_path, pData);
    vector<int> mask(pData.size(),0);

    // set the intrinsic parameters
    vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    inner <<intrinsic[0], intrinsic[1], intrinsic[2],
            intrinsic[3], intrinsic[4], intrinsic[5],
            intrinsic[6], intrinsic[7], intrinsic[8];

    // init the matrix of extrinsic, matrix of rotation and translation
    // keep this init value, or it is possible to get a local optimal result
    Eigen::Matrix3d R;  
    Eigen::Vector3d t;
    PnpTrans(pData, inner, R, t);

    Eigen::Quaterniond q(R); 
    // double ext[7]={q.x(),q.y(),q.z(),q.w(),t.x(),t.y(),t.z()};
    double ext[7]={q.x(),q.y(),q.z(),q.w(),0,0,0};

    Eigen::Map<Eigen::Quaterniond>  m_q = Eigen::Map<Eigen::Quaterniond>(ext);
    Eigen::Map<Eigen::Vector3d>     m_t = Eigen::Map<Eigen::Vector3d>(ext + 4);

    for(int i=0; i<2; ++i) {

        ceres::LocalParameterization * q_parameterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem problem;

        problem.AddParameterBlock(ext, 4, q_parameterization);
        problem.AddParameterBlock(ext + 4, 3);
    
        for(size_t j=0; j< pData.size(); ++j) {
            if(mask[j]==1)  {continue;} // 跳过误差大的点
            ceres::CostFunction *cost_function;
            cost_function = external_cali::Create(pData[j]);
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(5), ext, ext + 4);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        cout << summary.BriefReport() << endl;

        Eigen::Matrix3d rot = m_q.toRotationMatrix();
        writeExt(extrinsic_path, rot, m_t);
        cout << rot << endl;
        cout << m_t << endl;

        cout << "Use the extrinsic result to reproject the data" << endl;
        float error[2] = {0, 0};
        vector<float> extrinsic{(float)rot(0,0),(float)rot(0,1),(float)rot(0,2),(float)m_t(0),
                                (float)rot(1,0),(float)rot(1,1),(float)rot(1,2),(float)m_t(1),
                                (float)rot(2,0),(float)rot(2,1),(float)rot(2,2),(float)m_t(2)};
        getUVError(intrinsic, extrinsic,  pData,  error, error_threshold,mask);
        std::cout<<"mask: ";
        for_each(mask.begin(),mask.end(),[&](auto& a){ std::cout<<a<<"";});
        std::cout<<std::endl;
        cout << "u average error is: " << error[0] << endl;
        cout << "v average error is: " << error[1] << endl;
        if (error[0] + error[1] < error_threshold) {
            cout << endl << "The reprojection error smaller than the threshold, extrinsic result seems ok" << endl;
        }
        else {
            cout << endl << "The reprojection error bigger than the threshold, extrinsic result seems not ok" << endl;
        }
    }
    return 0;  
}


