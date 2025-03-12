#include <ros/ros.h>
#include <std_msgs/Float32.h> // 添加Float32消息类型的头文件
#include <iostream>
#include <thread>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <robot_control/curl_http_client.hpp>
#include <robot_control/ws_client.hpp>
#include <robot_control/subscribe_json.h>

// // 添加一个全局变量来存储最新的距离信息
// std::atomic<float> latest_distance(0.0);

// // 添加一个回调函数来处理接收到的距离信息
// void distanceCallback(const std_msgs::Float32::ConstPtr& msg)
// {
//     latest_distance.store(msg->data);
//     ROS_INFO("Received distance: %f", msg->data);
// }

void ws_client()
{
    // //1.实例化 HttpClient 客户端
    // HttpClient http_client;
    // // url 根据实际ip和端口 填写
    // http_client.set_url("http://192.168.1.102/apiUrl/");

    // //2. 链接http server，获取token
    // if (false == http_client.login_()){
    //     return ;
    // }


    //1.实例化 websocket 客户端
    WebSocketClient ws_client; 
    // url 根据实际ip和端口 填写
    std::string url = "ws://192.168.1.102:9090";

    ws_client.connect(url);
  
    // 等待2秒 等待连接成功  
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 2. 心跳数据定时器thread 一秒发送一次心跳信息,与服务端保存长连接的必要条件,收到的数据是二进制乱码
    std::thread([&ws_client ]() {
       string  heart_data = "{\"op\":\"ping\",\"timeStamp\":\"1\"}";
       while (1) {
            ws_client.send_message(heart_data);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
       }
     }).detach();
     
    //3. 需要订阅的话题数据，两种方式的数据，
    json virtual_joy_json = {
    {"op", "publish"},
    {"id", "publish:/run_management/virtual_joy"},
    {"topic", "/run_management/virtual_joy"},
    {"msg", {
        {"linear", {
            {"x", 0.3},
            {"y", 0},
            {"z", 0}
        }},
        {"angular", {
            {"x", 0}, 
            {"y", 0},
            {"z", 0.2}
        }}
    }}
    };

    ws_client.send_message(virtual_joy_json.dump());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

    // while(1) {
    //     // 获取最新的距离信息
    //     float dis_json = 1;
    //     printf("dis_json = %f\r\n",dis_json);
    //     json virtual_joy_json = {
    //         {"op", "publish"},
    //         {"id", "publish:/run_management/virtual_joy"},
    //         {"topic", "/run_management/virtual_joy"},
    //         {"msg", {
    //             {"linear", {
    //                 {"x", 0},
    //                 {"y", 0},
    //                 {"z", 0}
    //             }},
    //             {"angular", {
    //                 {"x", 0}, 
    //                 {"y", 0},
    //                 {"z", dis_json}
    //             }}
    //         }}
    //     };
    //     ws_client.send_message(virtual_joy_json.dump());

    //     // 每100ms更新一次
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }//堵塞作用，用于循环监听数据
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_control_node");
    ros::NodeHandle nh;
    // 创建一个ROS订阅者来接收距离信息
    // ros::Subscriber distance_sub = nh.subscribe("/yolov5/distance", 1, distanceCallback);
    // 创建一个单独的线程来运行ws_client函数
    std::thread ws_thread(ws_client);

    // 在主线程中运行ROS循环
    ros::spin();

    // 等待ws_thread结束（实际上不会发生，因为ws_client中有一个无限循环）
    ws_thread.join();
  

    return 0;
}