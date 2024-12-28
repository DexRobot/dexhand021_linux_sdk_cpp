#pragma once
#include <fstream>
#include <filesystem>
#include"CANWrapper.h"
#include<arpa/inet.h>

enum class Channel {
    Can0=0,
    Can1=1
};

enum class Feedback_methods{
    cycle=0x01,
    enquire=0x02,
    change=0x03
};




struct status_news{
    uint8_t fingerId;
    uint16_t Motor1_current;
    uint16_t Motor1_speed;
    uint16_t Motor1_location;
    uint16_t Motor2_current;
    uint16_t Motor2_speed;
    uint16_t Motor2_location;
    uint16_t Angle_sensor1;
    uint16_t Angle_sensor2;
    uint32_t Pressure_sensor_Normal_force;
    uint32_t Pressure_sensor_Tangential_force;
    uint32_t ps_1_ps;
    int temperature;
};

class Dexterous_hands
{
public:
    Dexterous_hands();
    bool start();
    bool stop();
    bool Clear_Error(FingerID fingerId,Channel channel);
    bool get_sdk_version(FingerID id,Channel channel,unsigned char* version);
    bool Open_Status_Feedback(FingerID fingerId, Channel channel,  Feedback_methods Feedback_methods,  U8 Feedback_cycles);
    bool Close_Status_Feedback(FingerID fingerId, Channel channel,  Feedback_methods Feedback_methods,  U8 Feedback_cycles);
    bool Degree_Control_mode(FingerID fingerId, Channel channel, JointMotor motor,  short proximal,  short distal);
    bool Unable_Control_mode(FingerID fingerId, Channel channel, JointMotor motor);
    bool Limit_Hall_Control_mode(FingerID fingerId,Channel channel, JointMotor motor,  short proximal,  short distal);
    status_news get_status_data(Channel channel);
    ~Dexterous_hands();
    class imp ;
    std::unique_ptr<imp> _pImpl;
};

