<!--
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-11-07 19:56:59
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-11-07 20:05:34
 -->
# ROS Package for indemind stereo camera with Benewake TF_mini_plus
## ros nodes
This ros package contains two sensor publisher nodes and one dataset recorder node. All sensor publisher nodes must be invoked with root go give it permission of accessing hardware interfaces. 
* tf_mini publisher:
```sh
sudo -s
rosrun sensor_module_ros tf_mini_node
```
* indemind stereo camera publisher:
```sh
sudo -s
rosrun sensor_module_ros indemind_vi_node
```
* dataset recorder:
```
rosrun sensor_module_ros record_node
```