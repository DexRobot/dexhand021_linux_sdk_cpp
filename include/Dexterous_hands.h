#pragma once
#include "CANWrapper.h"

enum class Channel {
    Can0=0,
    Can1=1
};

enum class Feedback_methods{
    cycle=0x01,
    enquire=0x02,
    change=0x03
};
enum class Feedback_cycles{
    on=0x01,
    off=0x00
};


enum class ERROR_CODE {
    ERR_SUCC,               
    ERR_INVALID_HANDLER,    
    ERR_INVALID_PARAMETER,  //��Ч�Ĳ���
    ERR_COMMUNICATION_ERR,  //ͨ�Ŵ���
    ERR_KINE_INVERSE_ERR,   //���ʧ��
    ERR_EMERGENCY_PRESSED,  //��ͣ��û���ɿ�
    ERR_NOT_POWERED,        //������δ�ϵ�
    ERR_NOT_ENABLED,        //������δʹ��
    ERR_PROGRAM_IS_RUNNING, //������������
    ERR_CANNOT_OPEN_FILE,   //���ļ�ʧ��
    ERR_MOTION_ABNORMAL,    //�˶������з����쳣
    ERR_VALUE_OVERSIZE,     //Ԥ���ڴ治��
    OPEN_CAN_FAILED,         //�����豸ʧ��
    CLOSE_CAN_FAILED        //�ر��豸ʧ��
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
    ERROR_CODE start();
    ERROR_CODE stop();
    bool Clear_Error(FingerID fingerId,Channel channel);
    bool get_sdk_version(U8 channel, FingerID id);
    bool Open_Status_Feedback(FingerID fingerId, Channel channel,  Feedback_methods Feedback_methods,  Feedback_cycles Feedback_cycles);
    bool Close_Status_Feedback(FingerID fingerId, Channel channel,  Feedback_methods Feedback_methods,  Feedback_cycles Feedback_cycles);
    bool Degree_Control_mode(FingerID fingerId, Channel channel, JointMotor motor,  short proximal,  short distal);
    bool Unable_Control_mode(FingerID fingerId, Channel channel, JointMotor motor);
    bool Limit_Hall_Control_mode(FingerID fingerId,Channel channel, JointMotor motor,  short proximal,  short distal);
    bool set_SDK_filepath(std::string url);
    status_news get_status_data(Channel channel);
    ~Dexterous_hands();
    class imp ;
    std::unique_ptr<imp> _pImpl;
};

