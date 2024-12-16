#pragma once
#include <vector>
#include <thread>
#include <memory>
#include <any>
#include <unordered_map>
#include <string>
#include <optional>
#include <ranges>
//#include <sstream>
#include <string.h>

#include "zcan.h"
#include "canframe.h"



enum class  WINMessages {
    CONFIGURATION_ERROR = 0x007,
    COMM_PROTOCOL_CHANGE = 0x008,
    COMM_DEVICE_CHANGE = 0x009
};

enum class  ZCAN_DeviceType
{
    UNKNOWN = 0,
    ZCAN_PCI5121 = 1,
    ZCAN_PCI9810 = 2,
    ZCAN_USBCAN1 = 3,
    ZCAN_USBCAN2 = 4,
    ZCAN_PCI9820 = 5,
    ZCAN_CAN232 = 6,
    ZCAN_PCI5110 = 7,
    ZCAN_CANLITE = 8,
    ZCAN_ISA9620 = 9,
    ZCAN_ISA5420 = 10,
    ZCAN_PC104CAN = 11,
    ZCAN_CANETUDP = 12,
    ZCAN_CANETE = 12,
    ZCAN_DNP9810 = 13,
    ZCAN_PCI9840 = 14,
    ZCAN_PC104CAN2 = 15,
    ZCAN_PCI9820I = 16,
    ZCAN_CANETTCP = 17,
    ZCAN_PCIE_9220 = 18,
    ZCAN_PCI5010U = 19,
    ZCAN_USBCAN_E_U = 20,
    ZCAN_USBCAN_2E_U = 21,
    ZCAN_PCI5020U = 22,
    ZCAN_EG20T_CAN = 23,
    ZCAN_PCIE9221 = 24,
    ZCAN_WIFICAN_TCP = 25,
    ZCAN_WIFICAN_UDP = 26,
    ZCAN_PCIe9120 = 27,
    ZCAN_PCIe9110 = 28,
    ZCAN_PCIe9140 = 29,
    ZCAN_USBCAN_4E_U = 31,
    ZCAN_CANDTU_200UR = 32,
    ZCAN_CANDTU_MINI = 33,
    ZCAN_USBCAN_8E_U = 34,
    ZCAN_CANREPLAY = 35,
    ZCAN_CANDTU_NET = 36,
    ZCAN_CANDTU_100UR = 37,
    ZCAN_PCIE_CANFD_100U = 38,
    ZCAN_PCIE_CANFD_200U = 39,
    ZCAN_PCIE_CANFD_400U = 40,
    ZCAN_USBCANFD_200U = 41,
    ZCAN_USBCANFD_100U = 42,
    ZCAN_USBCANFD_MINI = 43,
    ZCAN_CANFDCOM_100IE = 44,
    ZCAN_CANSCOPE = 45,
    ZCAN_CLOUD = 46,
    ZCAN_CANDTU_NET_400 = 47,
    ZCAN_CANFDNET_TCP = 48,
    ZCAN_CANFDNET_200U_TCP = 48,
    ZCAN_CANFDNET_UDP = 49,
    ZCAN_CANFDNET_200U_UDP = 49,
    ZCAN_CANFDWIFI_TCP = 50,
    ZCAN_CANFDWIFI_100U_TCP = 50,
    ZCAN_CANFDWIFI_UDP = 51,
    ZCAN_CANFDWIFI_100U_UDP = 51,
    ZCAN_CANFDNET_400U_TCP = 52,
    ZCAN_CANFDNET_400U_UDP = 53,
    ZCAN_CANFDBLUE_200U = 54,
    ZCAN_CANFDNET_100U_TCP = 55,
    ZCAN_CANFDNET_100U_UDP = 56,
    ZCAN_CANFDNET_800U_TCP = 57,
    ZCAN_CANFDNET_800U_UDP = 58,
    ZCAN_USBCANFD_800U = 59,
    ZCAN_PCIE_CANFD_100U_EX = 60,
    ZCAN_PCIE_CANFD_400U_EX = 61,
    ZCAN_PCIE_CANFD_200U_MINI = 62,
    ZCAN_PCIE_CANFD_200U_M2 = 63,
    ZCAN_CANFDDTU_400_TCP = 64,
    ZCAN_CANFDDTU_400_UDP = 65,
    ZCAN_CANFDWIFI_200U_TCP = 66,
    ZCAN_CANFDWIFI_200U_UDP = 67,
    ZCAN_CANFDDTU_800ER_TCP = 68,
    ZCAN_CANFDDTU_800ER_UDP = 69,
    ZCAN_CANFDDTU_800EWGR_TCP = 70,
    ZCAN_CANFDDTU_800EWGR_UDP = 71,
    ZCAN_CANFDDTU_600EWGR_TCP = 72,
    ZCAN_CANFDDTU_600EWGR_UDP = 73,
    ZCAN_CANFDDTU_CASCADE_TCP = 74,
    ZCAN_CANFDDTU_CASCADE_UDP = 75,
    ZCAN_USBCANFD_400U = 76,
    ZCAN_CANFDDTU_200U = 77,
    ZCAN_ZPSCANFD_TCP = 78,
    ZCAN_ZPSCANFD_USB = 79,
    ZCAN_CANFDBRIDGE_PLUS = 80,
    ZCAN_CANFDDTU_300U = 81
};

enum class  MotorControlMode {
    ZERO_TORQUE_MODE = 0x00,
    CURRENT_CONTROL_MODE = 0x11,
    SPEED_CONTROL_MODE = 0x22,
    HALL_POSITION_CONTROL_MODE = 0x33,
    CASCADED_PID_CONTROL_MODE = 0x44,
    limit_Hall_Control_mode = 0x55
};

enum class  JointMotor {
    DIST = 0x01,
    PROX = 0x02,
    ALL = 0x03
};

struct Joint{
     int motorID;     // ���ID
     int mode;        // ģʽ
     int current;     // ����
     int speed;       // �ٶ�
     int position;    // λ��
     int error;       // ����
     int minDegree;
     int maxDegree;
};


enum class  MotorBaseID {
    TX_BASE_ID = 0x20,
    RX_BASE_ID = 0x50
};

enum class  FingerJointInstID
{
    THUMB_ROTATION = 0,
    THUMB_DIP,
    THUMB_SPREAD,  /// palm proximal?
    FINGER_SPREAD, /// palm distal?
    INDEX_PIP,
    INDEX_DIP,
    MIDDLE_PIP,
    MIDDLE_DIP,
    RING_PIP,
    RING_DIP,
    PINKY_PIP,
    PINKY_DIP
};

enum class  JOINT_ID
{
    // Def left hand struct
    LEFT_THUMB_PROXIMAL = 0X51,
    LEFT_THUMB_DISTAL = 0X91,

    LEFT_PALM_ROLL = 0X52,
    LEFT_PALM_OPEN = 0X92,

    LEFT_INDEX_PROXIMAL = 0X53,
    LEFT_INDEX_DISTAL = 0X93,

    LEFT_MIDDLE_PROXIMAL = 0X54,
    LEFT_MIDDLE_DISTAL = 0X94,

    LEFT_RING_PROXIMAL = 0X55,
    LEFT_RING_DISTAL = 0X95,

    LEFT_PINKY_PROXIMAL = 0X56,
    LEFT_PINKY_DISTAL = 0X96,

    // Def right hand struct
    RIGHT_THUMB_PROXIMAL = 0X57,
    RIGHT_THUMB_DISTAL = 0X97,

    RIGHT_PALM_ROLL = 0X58,
    RIGHT_PALM_OPEN = 0X98,

    RIGHT_INDEX_PROXIMAL = 0X59,
    RIGHT_INDEX_DISTAL = 0X99,

    RIGHT_MIDDLE_PROXIMAL = 0X5A,
    RIGHT_MIDDLE_DISTAL = 0X9A,

    RIGHT_RING_PROXIMAL = 0X5B,
    RIGHT_RING_DISTAL = 0X9B,

    RIGHT_PINKY_PROXIMAL = 0X5C,
    RIGHT_PINKY_DISTAL = 0X9C
};

struct FingerMessage_CANFD
{
    FingerMessage_CANFD() { 

    }
    FingerMessage_CANFD(const FingerMessage_CANFD& other) {
        this->delay = other.delay;
        this->ctl_mode = other.ctl_mode;
        this->channel = other.channel;
        this->fingerId = other.fingerId;
        this->data = other.data;
    }
    int delay;  // milliseconds of delay to execute
    unsigned char ctl_mode;//�������ģʽ
    unsigned short channel;//ͨ��
    int fingerId;//��ָID
    ZCAN_FD_MSG data;//�
};

enum class  FingerID
{
    NONE_ID = 0x00,
    LEFT_THUMB = 0x01,
    LEFT_PALM = 0x02,
    LEFT_INDEX = 0x03,
    LEFT_MIDDLE = 0x04,
    LEFT_RING = 0x05,
    LEFT_PINKY = 0x06,

    RIGHT_THUMB = 0x07,
    RIGHT_PALM = 0x08,
    RIGHT_INDEX = 0x09,
    RIGHT_MIDDLE = 0x0A,
    RIGHT_RING = 0x0B,
    RIGHT_PINKY = 0x0C,

    ENTIRE_HAND = 0x0D
};

struct FingerInstructionData
{
    unsigned short channel;
    JointMotor motor;
    FingerID finger;
    unsigned short proximal;
    unsigned short distal;

    FingerInstructionData(unsigned short chn, JointMotor m, FingerID f, unsigned short p, unsigned short d);
   
};

struct FingerJointPositionCoef
{
    double gain;
    double bias;
    double lw_bound;
    double up_bound;
};

class CANFD_Instructions_BaseID
{
public:
    const unsigned int QueryTx = 0x0;//����
    const unsigned int QueryRx = 0x80;//����ָ��
    const unsigned int ControlTx = 0x100;
    const unsigned int StatusRx = 0x180;
    const unsigned int UpgradeRx = 0x300;
    const unsigned int StartupRx = 0x400;
    const unsigned int MarkRx = 0x500;
    const unsigned int ErrorRx = 0x600;
    const unsigned int ParamSetRx = 0x701;
    const unsigned int MarkConfirmRx = 0x800;
    const unsigned int ParamGetRx = 0x801;
    const unsigned int IDConfirmRx = 0x901;
};

enum class  ResponseMode
{
    REPEAT = 1,
    ASK,
    UPDATE
};

enum class  ErrorType
{
    None_Error = 0x00,
    M1_OverBound = 0x01,
    M2_OverBound = 0x02,
    MotorsError = 0xEE,
    MotorOverHeat = 0xFF,
};

enum class  MotorErrorCode
{

    CRT_OVERLOAD = 0x0001,
    HALL_ERROR = 0x0002,
    ROLL_BLOCKED = 0x0004,
    ADC_ERROR = 0x0008,
};

enum class  BootLoaderStage
{
    UNKNOWN = 0,
    Stuck_In_Bootloader,
    StartUp_Step1,
    StartUp_ToWork,
    StartUp_Upgrade,
    Ready_For_Upgrade,
    Upgrade_CRC_Accepted,
    Old_Erased,
    Upgrading,
    Upgrade_Done,
    Upgrade_Failed,
};

struct BootLoaderRxMessage
{
    FingerID fingerId;
    BootLoaderStage msgType;
    std::vector<unsigned char> buffer;
    bool succeeded;
};

struct StartupFirmVerRptMessage
{
    FingerID fingerId;
    int firmVersion;
};

 struct MotorErrorMsg
{
    ErrorType errorType;
    JointMotor motorId;
    MotorErrorCode errorCode;
    std::string errorMsg;

    MotorErrorMsg(JointMotor motorId);

};

 struct ErrorMessageRx
 {
     FingerID fingerId;
     std::shared_ptr<MotorErrorMsg> m1Error;
     std::shared_ptr<MotorErrorMsg> m2Error;

     ErrorMessageRx(FingerID fgrId);

 };

 struct SensorParam
 {
     int fingerId;
     int joint1MinADC;
     int joint2MinADC;
     int joint1MaxADC;
     int joint2MaxADC;
     double stopPressure1;
     double stopPressure2;
     unsigned short position1P;
     unsigned short position2P;
     unsigned short position1I;
     unsigned short position2I;
     unsigned short position1D;
     unsigned short position2D;
 };

 struct PressureSensorParam
 {
     int fingerId;
     double relaxMin;
     double relaxMax;
     double clenchMin;
     double clenchMax;
 };

 enum class  ParamMark_CMDID
 {
     DRIVER_MARK_MIN = 0x56,
     DRIVER_MARK_MAX = 0x57,
 };

 struct DriverMarkRx
 {
     unsigned char fingerId;  // fingerId
     ParamMark_CMDID markCmdId; // command ID
     JointMotor motorId;
     unsigned char result; // mark result
 };

 enum class  ParamSet_CMDID
 {
     DriverID = 0,
     DriverType,
     DriverReset,
     DriverCanType,
     UNKNOWN_1 = -601
 };

 struct DriverParamSetRx
 {
     unsigned char id;        // Finger ID/Driver ID
     ParamSet_CMDID cmdId; // Command ID
     unsigned char result;
 };

 struct MotorStatus
 {
     JointMotor id; // motor Id, 1 for proximal, 2 for distal
     int current;   // mA
     int16_t position;  // number of rounds of motor
     int speed;     // rpm
     unsigned char error;    // Error code
 };

 struct FingerRxStatus
 {
     unsigned char   channelId;
     unsigned char   fingerId;
     int    current1;
     int16_t  position1;
     int    speed1;
     int    current2;
     int16_t  position2;
     int    speed2;
     float  degree1;  // value of degree sensor 1
     float  degree2;  // value of degree sensor 2
     float nForce;  // value of normal force on pressure sensor
     int    nForceDelta;  // delta of normal force on pressure sensor
     float tForce;  // value of tangencial force on pressure sensor
     int    tForceDelta;  // delta of tangencial force on pressure sensor
     int    tForceAngle;  // degree of tangencial force on pressure sensor
     int    approaching;
     int    temperature;
     int    adc1;
     int    adc2;
     unsigned long  timestamp;

     FingerRxStatus(unsigned char chnId, unsigned char fid, unsigned char ts);

     bool operator==(FingerRxStatus other);
     
     /*int GetHashCode() 
     {
         
     }*/
 };

 enum class  DexCommandType
 {
     DRIVER_PRM_GET = 0x1,
     DRIVER_PRM_SET,
     GLOBAL_PRM_SET,
     DRIVER_CRT_CTL = 0x11,  // ��������
     DRIVER_SPD_CTL = 0x22,  // �ٶȿ���
     DRIVER_HAL_CTL = 0x33,  // ��������
     DRIVER_DEG_CTL = 0x44,  // �Ƕ�(���Ȧ��)��
 };

 enum class  DexParamType
 {
     DRIVER_ID = 0x1,
     DRIVER_PARM_MIN = 0xD,
     DRIVER_PARM_MAX = 0x3A,
     RESPONSE_MODE = 0x74,
 };



 class Protocols {
 public:
     Protocols();
     enum class  ActionID{
         Bend = 1,
         Relax,
         Split,
         Close,
         Roll,
         MotorTest,
         Custom,
     };

     struct ActionPerformance
     {
         ActionID aciontId;
         std::string actionName;
         unsigned long startTs;
         unsigned long stopTs;
         unsigned long duration;

         ActionPerformance(ActionID id, std::string name, unsigned long start, unsigned long end);
     };

     enum class  FirmwareUpgradeStep
     {
         BOOT_CONFIRM = 0x01,
         ACCEPTED = 0x01,
         ERASED = 0x02,
         WRITINT = 0x03,
         DONE = 0x04,
     };

     enum class  FirmwareUpgradeStepRes
     {
         BOOT1,
         BOOT2,
         BOOT_CONFIRM,
         CRC_ACCEPTED,
         ERASED,
         WRITING,
         DONE,
         FAILED
     };


     long CurrentTimestamp();//����ǰʱ�����л���Unix��Ԫ���������ĺ�����

     std::string TimestampToDateTime(unsigned long ts);//��Unixʱ��������л��ɱ���ʱ��

     ZCAN_DeviceType DeviceNameToType(const std::string deviceName);

     void build_finger_canfd_frame(int TX_BASE_ID,U8 chn, int finger_id, int8_t motor_enabled, int16_t proximal_data, int16_t distal_data,
                              ZCAN_FD_MSG *can, MotorControlMode control_mode );


     std::unordered_map<std::string, FingerJointInstID> URDF_FingerInstMap;
     std::unordered_map<FingerID, unsigned int> FingerChannelMap;
 private:
     //std::unordered_map<unsigned char, ZCAN_ReceiveFD_Data> markedParams;
     //std::unordered_map<unsigned char, ZCAN_ReceiveFD_Data> driverParams;
     std::unordered_map<ParamSet_CMDID, std::vector<unsigned char>> ParamSet_REPHeaders;
     std::unordered_map<ParamSet_CMDID, std::vector<unsigned char>> ParamSet_CMDHeaders;
     std::unordered_map<FirmwareUpgradeStepRes, std::vector<unsigned char>> DriverFirmwareUpgMsg;
     std::unordered_map<unsigned short, std::vector<unsigned char>> DriverStartupMsg; 

     ParamSet_CMDID DetermineCMD_ParamSet(std::vector<unsigned char> header);
     ParamSet_CMDID DetermineREP_ParamSet(std::vector<unsigned char> header);
     std::unordered_map<std::string, ZCAN_DeviceType> ZCAN_enum_To_Str;
 };
