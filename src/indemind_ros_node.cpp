/*
 * @Description: Indemind双目惯性模组数据读取与发布节点
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-11-01 14:28:33
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-11-07 15:59:26
 */
#include "common_headers.h"
using namespace indem;
// =============================== 全局变量 ===================================//
ros::Time imu_start_time; //IMU消息起始时间
ros::Time camera_start_time; //图像消息起始时间

int imu_id = 0; //IMU消息编号
int image_l_id = 0; //左相机图像标号
int image_r_id = 0; //右相机图像编号
// 消息Publisher
ros::Publisher imu_pub;
image_transport::Publisher image_l_pub;
image_transport::Publisher image_r_pub;
ros::Publisher camera_info_pub;
// 使用相机标识
enum CameraFlag{
    CAM_LEFT = 0,
    CAM_RIGHT = 1,
    CAM_BOTH,
    CAM_DEFAULT = CAM_LEFT
};

// =================================== 全局回调函数 ============================== //
/**
 * @brief IMU的回调函数, 只进行IMU消息的生成和发布. 根据官方文档,请勿于此函数中进行负责
 *      操作,否则可能引起数据丢失.该函数的调用频率与设定的IMU数据采集频率一致.
 * @param data : IMU数据结构体;
 */
void ImuCallback(IMUData* data){
    sensor_msgs::Imu imu_msg;
    // 如果是第一条消息,则初始化时戳
    // 硬件的timeStamp是以ms为单位的, ros::Duration是以s为单位,需要转换
    if(imu_id == 0){
        imu_start_time = ros::Time::now() - ros::Duration(data->_timeStamp / 1000.);
    }
    // 构建消息头
    imu_msg.header.stamp = imu_start_time + ros::Duration(data->_timeStamp / 1000.);
    // ROS_INFO("IMU_Stamp: %lf", data->_timeStamp);
    imu_msg.header.seq = imu_id++;
    // 构建量测消息
    imu_msg.linear_acceleration.x = data->_acc[0];
    imu_msg.linear_acceleration.y = data->_acc[1];
    imu_msg.linear_acceleration.z = data->_acc[2];

    imu_msg.angular_velocity.x = data->_gyr[0];
    imu_msg.angular_velocity.y = data->_gyr[1];
    imu_msg.angular_velocity.z = data->_gyr[2];
    // 发布消息
    imu_pub.publish(imu_msg);
    return;
}

/**
 * @brief IMU的回调函数, 只进行IMU消息的生成和发布. 根据官方文档,请勿于此函数中进行负责
 *      操作,否则可能引起数据丢失.该函数的调用频率与设定的IMU数据采集频率一致.
 * @param data 
 */
void CameraCallback(cameraData* data){
    if(image_l_id == 0){
        camera_start_time = ros::Time::now() - ros::Duration(data->_timeStamp / 1000.);
    }
    // -------------------- 获取双目相机图像 -------------------------- //
    int img_height = data->_height;
    int img_width = data->_width;
    cv::Mat img_stereo(img_height, img_width, CV_8UC1, data->_image);
    
    // --------------------构建左相机图像消息 -------------------------- //
    cv::Mat img_l(img_stereo, cv::Rect(0, 0, img_width / 2, img_height));
    sensor_msgs::ImagePtr img_msg_l;
    std_msgs::Header img_header;
    // 构建消息头
    img_header.stamp = camera_start_time + ros::Duration(data->_timeStamp / 1000.);
    img_header.seq = image_l_id++;
    // 将cv::Mat转换为ros图像消息
    img_msg_l = cv_bridge::CvImage(img_header, "mono8", img_l).toImageMsg();
    // 发布图像消息
    image_l_pub.publish(img_msg_l);
    
    // ------------------ 构建右相机图像消息 -------------------------- //
    cv::Mat img_r(img_stereo, cv::Rect(0, 0, img_width / 2, img_height));
    sensor_msgs::ImagePtr img_msg_r;
    // 构建消息头
    // img_header.stamp = camera_start_time + ros::Duration(data->_timeStamp / 1000.);
    // img_header.seq = image_r_id++;
    // 将cv::Mat转换为ros图像消息
    img_msg_r = cv_bridge::CvImage(img_header, "mono8", img_r).toImageMsg();
    // 发布图像消息
    image_r_pub.publish(img_msg_r);
    
    // ------------------ 释放资源 --------------------------------- //
    img_stereo.release();
    img_l.release();
    img_r.release();   
}

// ===================================== 全局工具函数 ======================================================= //
/**
 * @brief 
 * 
 * @param image_width ： 图像宽度 
 * @param image_height ： 图像高度
 * @param imu_freq ： IMU数据采集频率
 * @param img_freq ： 图像采集频率
 * @param img_res ： 图像分辨率
 */
void paramsCheck(int& image_width, int& image_height, int& imu_freq, int& img_freq, enum IMAGE_RESOLUTION& img_res);

/**
 * @brief 
 * 
 * @param nh : ROS节点句柄
 * @param moduleParam ： 模组参数结构体 
 */
void uploadIMUParams(ros::NodeHandle& nh, ModuleParamInFlash<1>& moduleParam);
 
 /**
  * @brief 
  * 
  * @param nh ： ROS节点句柄
  * @param moduleParam ： 模组参数结构体 
  * @param cam : 需要获取参数的相机，CAM_LEFT, CAM_RIGHT或CAM_BOTH，默认为CAM_LEFT
  */
void uploadCameraParams(ros::NodeHandle& nh, ModuleParamInFlash<1>& moduleParam, enum CameraFlag cam=CAM_DEFAULT);
// ================================ 节点主函数 ================================ //

int main(int argc, char** argv){
    ros::init(argc, argv, "indemind_vi"); // 初始化ros节点
    ros::NodeHandle nh; // 初始化节点句柄
    // ---------------------- 初始化Publisher ------------------------------ //
    imu_pub = nh.advertise<sensor_msgs::Imu>("/indemind_vi_node/imu", 100);
    image_transport::ImageTransport it(nh);
    image_l_pub = it.advertise("/indemind_vi_node/image_left", 10);
    image_r_pub = it.advertise("/indemind_vi_node/image_right", 10);
    // ------------------------ 初始化API --------------------------------- //
    IDriverInterface *driver = DriverFactory();
    // ------------------- 从ROS参数服务器获取节点参数 ---------------------- //
    int image_width, image_height; //图像尺寸
    int imu_freq, img_freq; //IMU和图像读取频率
    enum IMAGE_RESOLUTION img_res = RESOLUTION_640; // 图像分辨率

    nh.getParam("/indemind_vi_node/img_width", image_width);
    nh.getParam("/indemind_vi_node/img_height", image_height);
    nh.getParam("/indemind_vi_node/img_freq", img_freq);
    nh.getParam("/indemind_vi_node/imu_freq", imu_freq);

    paramsCheck(image_width, image_height, imu_freq, img_freq, img_res);
    // -------------- 获取相机模组参数并上传ROS参数服务器 -------------------- //
    // 读取模组参数相关
    int version = 255;
    size_t paramSize = 0;
    unsigned char *module_info = new unsigned char[FLASH_MAX_SIZE];
    ModuleParamInFlash<1> moduleParam = {0};  
    if(!driver->GetModuleParams(version, module_info, paramSize)){
        ROS_ERROR("Get device parameters failed !");
    }
    else{
        memcpy(&moduleParam, module_info, paramSize);
    }
    // 上传模组参数
    uploadIMUParams(nh, moduleParam);
    uploadCameraParams(nh, moduleParam);
    // ------------------ 启动设备并设置传感器回调函数 ---------------------- //
    if(!driver->Open(imu_freq, img_freq)){
        ROS_ERROR("Open device failed !");
    }
    else{
        driver->SetIMUCallback(ImuCallback);
        driver->SetCameraCallback(CameraCallback);
    }

    while(ros::ok()){
        ros::spinOnce();
    }

    driver->Close();
    return 0;
}
/**
 * @brief 
 * 
 * @param image_width ： 图像宽度 
 * @param image_height ： 图像高度
 * @param imu_freq ： IMU数据采集频率
 * @param img_freq ： 图像采集频率
 * @param img_res ： 图像分辨率
 */
void paramsCheck(int& image_width, int& image_height, int& imu_freq, int& img_freq, enum IMAGE_RESOLUTION& img_res){
    if (image_width == 1280){
        img_res = RESOLUTION_1280;
        image_height = 800;
    }
    else if (image_width == 640){
        img_res = RESOLUTION_640;
        image_height = 400;
    }
    else {
        image_width = 640;
        image_height = 400;
        img_res = RESOLUTION_640;
        ROS_WARN("Unknown resolution, use default resolution: 640 x 400 !");
    }

    if (img_freq != 25 && img_freq != 50 && img_freq != 100 && img_freq != 200) {
        ROS_WARN("Unknown FPS, set to default FPS: 50 !");
        img_freq = 50;
    }
    if (img_freq == 200 && img_res != RESOLUTION_640) {
        img_freq = 100;
        ROS_WARN("200 FPS only support in resolution: 640 x 400 !");
    }

    if (imu_freq > 1000) {
        ROS_WARN("Maximum IMU frequency is 1000Hz, set to 1000Hz !");
        imu_freq = 1000;
    }
}

void uploadIMUParams(ros::NodeHandle& nh, ModuleParamInFlash<1>& moduleParam){
    // 将加速度计和陀螺补偿矩阵转为vector方便写入params服务器
    std::vector<double> acc_coefficients;
    std::vector<double> gyro_coefficients;
    for(int i=0;i<12;i++){
        acc_coefficients.push_back(moduleParam._parent._imu._Acc[i]);
        gyro_coefficients.push_back(moduleParam._parent._imu._Gyr[i]);
    }
    std::vector<double> T_bs;
    for(int i=0;i<16;i++){
        T_bs.push_back(moduleParam._parent._imu._T_BS[i]);
    }
    
    // 写入params服务器
    nh.setParam("/indemind_vi_node/imu_params/a_max", moduleParam._parent._imu._a_max); //加速度计量程
    nh.setParam("/indemind_vi_node/imu_params/sigma_ba", moduleParam._parent._imu._sigma_ba); //加速度计零偏方差
    nh.setParam("/indemind_vi_node/imu_params/sigma_a_c", moduleParam._parent._imu._sigma_a_c); //加速度计噪声方差
    nh.setParam("/indemind_vi_node/imu_params/sigma_aw_c", moduleParam._parent._imu._sigma_aw_c); //加速度计随机游走方差
    nh.setParam("/indemind_vi_node/imu_params/acc_coefficients", acc_coefficients); //加速度计补偿矩阵3x4
    
    nh.setParam("/indemind_vi_node/imu_params/g_max", moduleParam._parent._imu._g_max); //陀螺量程
    nh.setParam("/indemind_vi_node/imu_params/sigma_bg", moduleParam._parent._imu._sigma_bg); //陀螺零偏方差
    nh.setParam("/indemind_vi_node/imu_params/sigma_g_c", moduleParam._parent._imu._sigma_g_c); //陀螺噪声方差
    nh.setParam("/indemind_vi_node/imu_params/sigma_gw_c", moduleParam._parent._imu._sigma_gw_c); //陀螺随机游走方差
    nh.setParam("/indemind_vi_node/imu_params/gyro_coefficients", gyro_coefficients); //陀螺补偿矩阵3x4
   
    nh.setParam("/indemind_vi_node/imu_params/gravity", moduleParam._parent._imu._g); //重力矢量，用于还原加速度
    nh.setParam("/indemind_vi_node/imu_params/T_bs", T_bs);
}

void uploadCameraParams(ros::NodeHandle& nh, ModuleParamInFlash<1>& moduleParam, enum CameraFlag cam){
    std::vector<double> Tsc, R, P, K, D;
    // 写入左相机参数
    if(cam == CAM_LEFT || cam == CAM_BOTH){
        // 相机到传感器变换矩阵Tsc
        for(int i=0; i<16; i++){
            Tsc.push_back(moduleParam._parent._camera[0]._TSC[i]);
        }
        // 旋转矩阵R及内参矩阵K
        for(int i=0; i<9; i++){
            R.push_back(moduleParam._parent._camera[0]._R[i]);
            K.push_back(moduleParam._parent._camera[0]._K[i]);
        }
        // 投影矩阵P
        for(int i=0; i<12; i++){
            P.push_back(moduleParam._parent._camera[0]._P[i]);
        }
        // 鱼眼畸变系数D
        for(int i=0; i<4; i++){D.push_back(moduleParam._parent._camera[0]._D[i]);}

        nh.setParam("/indemind_vi_node/camera_left/Tsc", Tsc);
        nh.setParam("/indemind_vi_node/camera_left/R", R);
        nh.setParam("/indemind_vi_node/camera_left/K", K);
        nh.setParam("/indemind_vi_node/camera_left/P", P);
        nh.setParam("/indemind_vi_node/camera_left/D", D);
    }
    // 写入右相机参数
    if(cam == CAM_RIGHT || cam == CAM_BOTH){
        // 清空所有容器用于存放右相机参数
        Tsc.clear(); 
        R.clear(); P.clear(); 
        K.clear(); D.clear();    
        // 相机到传感器变换矩阵Tsc
        for(int i=0; i<16; i++){
            Tsc.push_back(moduleParam._parent._camera[1]._TSC[i]);
        }
        // 旋转矩阵R及内参矩阵K
        for(int i=0; i<9; i++){
            R.push_back(moduleParam._parent._camera[1]._R[i]);
            K.push_back(moduleParam._parent._camera[1]._K[i]);
        }
        // 投影矩阵P
        for(int i=0; i<12; i++){
            P.push_back(moduleParam._parent._camera[1]._P[i]);
        }
        // 鱼眼畸变系数D
        for(int i=0; i<4; i++){D.push_back(moduleParam._parent._camera[1]._D[i]);}

        nh.setParam("/indemind_vi_node/camera_right/Tsc", Tsc);
        nh.setParam("/indemind_vi_node/camera_right/R", R);
        nh.setParam("/indemind_vi_node/camera_right/K", K);
        nh.setParam("/indemind_vi_node/camera_right/P", P);
        nh.setParam("/indemind_vi_node/camera_right/D", D);
    }
}