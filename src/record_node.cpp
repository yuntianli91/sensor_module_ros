/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-11-06 10:58:29
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-11-07 16:31:04
 */
#include "common_headers.h"
#include <sensor_msgs/Range.h>
// ===================================== 全局变量 ================================== //
FILE *imu_fp, *ranger_fp;
bool init_flag = true;
int imu_count = 0;
int img_count = 0;

// ================================== 各消息回调函数 ================================ //
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    
    // 写入测量参数
    ROS_INFO("Recording IMU at %lu.", msg->header.stamp.toNSec());
    fprintf(imu_fp, "%lu,%lf,%lf,%lf,%lf,%lf,%lf\n", 
            msg->header.stamp.toNSec(),
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);     
    return;   
}

void cameraCallback(const sensor_msgs::ImageConstPtr& msg, int cam_id){
    cv::Mat img;
    size_t img_time = msg->header.stamp.toNSec();
    ROS_INFO("Recording image at %lu.", img_time);
    // cv::imshow("left", cv_bridge::toCvShare(msg, "mono8")->image);
    // cv::waitKey(1);

    string folder_name;
    cam_id == 0 ? folder_name = "img0/" : folder_name = "img1/";
    // cout << "./sensorDatasets/indemind/" + folder_name + std::to_string(img_time) + ".jpg";
    cv::imwrite("./sensorDatasets/indemind/" + folder_name + std::to_string(img_time) + ".jpg",
            cv_bridge::toCvShare(msg, "mono8")->image);
    // img_count++;
    return;
}

void rangerCallback(const sensor_msgs::Range::ConstPtr& msg){
    fprintf(ranger_fp, "%lu,%lf\n", msg->header.stamp.toNSec(), msg->range);
    return;
}

// =================================== 标定参数写入 ================================== //
void writeImuParamsYaml(ros::NodeHandle& nh);
void writeCameraParamsYaml(ros::NodeHandle& nh, int cam_id);
// =================================== 节点主函数 ==================================== //
int main(int argc, char** argv){
    // ---------------------- 初始化ROS节点 ----------------------------------- //
    ros::init(argc, argv, "record_node");
    ros::NodeHandle nh;
    // ------------ 打开imu和ranger数据文件，写入文件头 -------------------------- //
    system("rm -rf ./sensorDatasets/indemind");
    system("mkdir -p ./sensorDatasets/indemind/img0");
    // system("mkdir -p ./sensorDatasets/indemind/img1");
    system("mkdir -p ./sensorDatasets/indemind/imu");
    system("mkdir -p ./sensorDatasets/indemind/ranger");
    // 打开并创建IMU数据文件
    imu_fp = fopen("./sensorDatasets/indemind/imu/imu.csv", "w");
    if(imu_fp == NULL){
        ROS_ERROR("Failed to open imu file !");
    }
    else{
      fprintf(imu_fp, "time_stamp(ns),acc_x(g),acc_y(g),acc_z(g),omega_x(deg/s),omega_y(deg/s),omega_z(deg/s)\n");
    }
    //打开并创建ranger数据文件
    ranger_fp = fopen("./sensorDatasets/indemind/ranger/ranger.csv", "w");
    if(ranger_fp == NULL){
        ROS_ERROR("Failed to open range file !");
    }
    else{
        fprintf(ranger_fp, "time_stamp(ns),range(m)");
    }
    // -------------------------- 订阅消息 ------------------------------------- //
    ros::Subscriber imu_sub = nh.subscribe("/indemind_vi_node/imu", 1000, imuCallback);
    ros::Subscriber ranger_sub = nh.subscribe("/tf_mini_node/range", 1000, rangerCallback);
    int cam_id = 0;
    ros::Subscriber image_sub_left = nh.subscribe<sensor_msgs::Image>("/indemind_vi_node/image_left", 100, boost::bind(&cameraCallback, _1, cam_id));
    // cam_id = 1;
    // ros::Subscriber image_sub_right = nh.subscribe<sensor_msgs::Image>("/indemind_vi_node/image_right", 100, boost::bind(&cameraCallback, _1, cam_id));
    // -------------------------- 读取模组标定参数并写入文件 ---------------------- //
    writeImuParamsYaml(nh);
    writeCameraParamsYaml(nh, 0);
    // -------------------------- 启动ROS循环直到节点关闭------------------------ //
    ros::spin();
    // ros::shutdown();
    return 0;
}

void writeImuParamsYaml(ros::NodeHandle& nh){
    cv::FileStorage fs_calib; //写入YAML的类
    fs_calib.open("./sensorDatasets/indemind/imu/params.yaml", cv::FileStorage::WRITE);
    if(!fs_calib.isOpened()){ROS_ERROR("Unable to open imu params file !");}

    ROS_INFO("Writing IMU Parameters...");
    double param_temp;  
    nh.getParam("/indemind_vi_node/imu_params/a_max", param_temp);
    fs_calib << "a_max" << param_temp;
    nh.getParam("/indemind_vi_node/imu_params/sigma_a_c", param_temp);
    fs_calib << "sigma_a_c" << param_temp;
    nh.getParam("/indemind_vi_node/imu_params/sigma_ba", param_temp);
    fs_calib << "sigma_ba" << param_temp;
    nh.getParam("/indemind_vi_node/imu_params/sigma_aw_c", param_temp);
    fs_calib << "sigam_aw_c" << param_temp;
    
    nh.getParam("/indemind_vi_node/imu_params/g_max", param_temp);
    fs_calib << "g_max" << param_temp;
    nh.getParam("/indemind_vi_node/imu_params/sigma_g_c", param_temp);
    fs_calib << "sigma_g_c" << param_temp;
    nh.getParam("/indemind_vi_node/imu_params/sigma_bg", param_temp);
    fs_calib << "sigma_bg" << param_temp;
    nh.getParam("/indemind_vi_node/imu_params/sigma_gw_c", param_temp);
    fs_calib << "sigam_gw_c" << param_temp;
    
    // nh.getParam("/indemind_vi_node/imu_params/tau", param_temp);
    // fs_calib << "tau" << param_temp;
    nh.getParam("/indemind_vi_node/imu_params/g", param_temp);
    fs_calib << "g" << param_temp;
    // ---------------------------- 参数数组 ----------------------- //
    std::vector<double> vector_temp;

    nh.getParam("/indemind_vi_node/imu_params/T_bs", vector_temp);
    ROS_INFO("Size of vector_temp is %lu", vector_temp.size());
    cv::Mat T_BS(4, 4, CV_64F);
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            // ROS_INFO("Here%d and %d.", i, j);
            T_BS.at<double>(i, j) = vector_temp[4 * i + j];
            // ROS_INFO("%lf", vector_temp[4 * i + j]);
        }
    }
    fs_calib << "T_BS" <<T_BS;
    vector_temp.clear();

    ROS_INFO("Geting IMU matrix...");
    std::vector<double> vector_temp1;
    nh.getParam("/indemind_vi_node/imu_params/acc_coefficients", vector_temp);
    nh.getParam("/indemind_vi_node/imu_params/gyro_coefficients", vector_temp1); //加速度计补偿矩阵3x4
    cv::Mat AccParams(3, 4, CV_64F), GyroParams(3, 4, CV_64F);
    for(int i=0; i<3; i++){
        for(int j=0; j<4; j++){
            ROS_INFO("Here%d and %d.", i, j);
            AccParams.at<double>(i, j) = vector_temp[4 * i + j];
            GyroParams.at<double>(i, j) = vector_temp1[4 * i + j];
        }
    }

    fs_calib << "AccParams" << AccParams << "GyroParams" << GyroParams;
    fs_calib.release();

    ROS_INFO("Writing IMU Parameters Done !");
    return;
}

void writeCameraParamsYaml(ros::NodeHandle& nh, int cam_id){

    cv::FileStorage fs_calib;
    if(cam_id == 0){
        fs_calib.open("./sensorDatasets/indemind/img0/params.yaml", cv::FileStorage::WRITE);
    }
    else{
        fs_calib.open("./sensorDatasets/indemind/img1/params.yaml", cv::FileStorage::WRITE);
    }
    if(!fs_calib.isOpened()){ROS_ERROR("Unable to open imu params file !");}

    string topic_name;
    cam_id == 0 ? topic_name = "camera_left" : topic_name = "camera_right";

    ROS_INFO("Writing Camera Parameters...");
    std::vector<double> vector_temp;
    nh.getParam("/indemind_vi_node/" + topic_name + "/Tsc", vector_temp);
    cv::Mat TSC(4, 4, CV_64F); // 相机坐标系到IMU坐标系的坐标变换矩阵
    for (int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            TSC.at<double>(i, j) = vector_temp[4 * i + j];
        }
    }
    vector_temp.clear();
    
    std::vector<double> vector_temp1;
    nh.getParam("/indemind_vi_node/" + topic_name + "/R", vector_temp);
    nh.getParam("/indemind_vi_node/" + topic_name + "/K", vector_temp1);
    cv::Mat R(3, 3, CV_64F), K(3, 3, CV_64F); // 旋转矩阵和内存矩阵
    for (int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            R.at<double>(i, j) = vector_temp[3 * i + j];
            K.at<double>(i, j) = vector_temp1[3 * i + j];
        }
    }
    vector_temp.clear();
    
    nh.getParam("/indemind_vi_node/" + topic_name + "/P", vector_temp);
    cv::Mat P(3, 4, CV_64F); // 投影矩阵
    for (int i=0; i<3; i++){
        for(int j=0; j<4; j++){
            P.at<double>(i, j) = vector_temp[4 * i + j];
        }
    } 
    vector_temp.clear();
    
    nh.getParam("/indemind_vi_node/" + topic_name + "/D", vector_temp);
    cv::Mat D(1, 4, CV_64F); // 畸变系数
    for (int i=0; i<4; i++){
        D.at<double>(0, i) = vector_temp[i];
    }
    
    string cam_flag;
    cam_flag = (cam_id ==0) ? "lCam" : "rCam";
    
    fs_calib << cam_flag + "_instrinsic_matrix" << K
        << cam_flag + "_projection_matrix" << P
        << cam_flag + "_rotation_matrix"<< R
        << cam_flag + "_distortion_matrix" << D
        << cam_flag + "_transformation_matrix"<< TSC;

    ROS_INFO("Writing Camera Parameters Done !");
    return;

}