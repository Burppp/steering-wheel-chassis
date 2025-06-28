#ifndef OBJECT_DICTIONARY_H
#define OBJECT_DICTIONARY_H

#include <stdint.h>
#include <stdbool.h>

// 对象字典索引枚举
typedef enum {
    // 模式及控制
    OD_INDEX_CONTROL_WORD = 0x6040,    // 控制字
    OD_INDEX_STATUS_WORD = 0x6041,     // 状态字
    OD_INDEX_MODE_OF_OPERATION = 0x6060, // 运行模式
    OD_INDEX_ENCODER_DATA_RESET = 0x2690, // 编码器多圈数据复位

    // 测量数据
    OD_INDEX_ACTUAL_POSITION = 0x6063,   // 实际位置
    OD_INDEX_ACTUAL_CURRENT = 0x6078,   // 实际电流（I_q）
    OD_INDEX_DIGITAL_INPUT = 0x60FD,    // 输入口状态
    OD_INDEX_ACTUAL_VELOCITY = 0x606C,  // 实际速度

    // 目标对象
    OD_INDEX_INVERT_DIR = 0x607E,       //速度极性
    OD_INDEX_TARGET_POS = 0x607A,       //目标位置
    OD_INDEX_PROFILE_SPEED = 0x6081,    //梯形速度
    OD_INDEX_TARGET_SPEED = 0x60FF,     //目标速度
    OD_INDEX_MAX_SPEED_RPM = 0x6080,    //最大速度
    OD_INDEX_PROFILE_ACCELERATION = 0x6083, //加速度
    OD_INDEX_PROFILE_DECELERATION = 0x6084, //减速度
    OD_INDEX_TARGET_TORQUE = 0x6071,       //目标转矩
    OD_INDEX_GROUP_CURRENT_LOOP = 0x60F6,  //电流环参数     Kcp:1,Kci:2,Speed_Limit_Factor:3,CMD_q:8,CMD_d:9
    OD_INDEX_CMD_Q_MAX = 0x6073,           //电流环最大电流

    // 速度环参数
    OD_INDEX_GROUP_SPEED_LOOP = 0x60F9,     //Kvp:1,Kvi:2,,Speed_Mode:6,Speed_Error:1C，Speed_Fb_N:5

    // 位置环参数
    OD_INDEX_GROUP_POSITION_LOOP = 0x60FB,  //Kpp:1,K_Velocity_FF:2,K_Acc_FF:3,Pos_Filter_N:5
	
	//TPDO配置
	OD_INDEX_TPDO_VALUE = 0x1A00,
} OD_Index;

typedef enum{
    OD_SUBINDEX_DEFAULT = 0x00,
    OD_SUBINDEX_1 = 0x01,
    OD_SUBINDEX_2 = 0x02,
    OD_SUBINDEX_3 = 0x03,
    OD_SUBINDEX_4 = 0x04,
    OD_SUBINDEX_5 = 0x05,
    OD_SUBINDEX_6 = 0x06,
    OD_SUBINDEX_7 = 0x07,
    OD_SUBINDEX_8 = 0x08,
    OD_SUBINDEX_9 = 0x09,
    OD_SUBINDEX_10 = 0x0A,
    OD_SUBINDEX_11 = 0x0B,
    OD_SUBINDEX_12 = 0x0C,
    OD_SUBINDEX_13 = 0x0D,
    OD_SUBINDEX_14 = 0x0E,
    OD_SUBINDEX_15 = 0x0F,
    OD_SUBINDEX_16 = 0x10,
    OD_SUBINDEX_17 = 0x11,
    OD_SUBINDEX_18 = 0x12,
    OD_SUBINDEX_19 = 0x13,
    OD_SUBINDEX_20 = 0x14,
    OD_SUBINDEX_21 = 0x15,
    OD_SUBINDEX_22 = 0x16,
    OD_SUBINDEX_23 = 0x17,
    OD_SUBINDEX_24 = 0x18,
    OD_SUBINDEX_25 = 0x19,
    OD_SUBINDEX_26 = 0x1A,
    OD_SUBINDEX_27 = 0x1B,
    OD_SUBINDEX_28 = 0x1C,
    OD_SUBINDEX_29 = 0x1D,
    OD_SUBINDEX_30 = 0x1E,
    OD_SUBINDEX_31 = 0x1F,
    OD_SUBINDEX_32 = 0x20
}OD_SubIndex;

// 对象字典项的数据类型枚举
typedef enum {
    OD_TYPE_BOOLEAN,
    OD_TYPE_INT8,
    OD_TYPE_INT16,
    OD_TYPE_INT32,
    OD_TYPE_UINT8,
    OD_TYPE_UINT16,
    OD_TYPE_UINT32,
    OD_TYPE_FLOAT32,
    OD_TYPE_STRING
} OD_DataType;

// 对象字典项的结构体（链表节点）
typedef struct OD_Entry {
    uint16_t index;          // 主索引
    uint8_t subindex;        // 子索引
    OD_DataType dataType;    // 数据类型
    void* data;              // 数据指针
    bool accessRead;         // 读权限
    bool accessWrite;        // 写权限
    struct OD_Entry* next;   // 指向下一个节点的指针
} OD_Entry;

// 对象字典结构体
typedef struct {
    OD_Entry* head;          // 链表头指针
    uint16_t entryCount;     // 条目数量
} ObjectDictionary;

// 函数声明
ObjectDictionary* OD_Create(void);
void OD_Destroy(ObjectDictionary* od);
bool OD_AddEntry(ObjectDictionary* od, OD_Entry* entry);
bool OD_RemoveEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex);
OD_Entry* OD_GetEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex);
bool OD_SetValue(ObjectDictionary* od, uint16_t index, uint8_t subindex, void* value);
bool OD_GetValue(ObjectDictionary* od, uint16_t index, uint8_t subindex, void* value);
void OD_PrintAll(ObjectDictionary* od);  // 打印所有条目（用于调试）
bool OD_User_AddEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex, OD_DataType dataType, void* data, bool accessRead, bool accessWrite);
bool OD_User_Init(ObjectDictionary* od);

#endif // OBJECT_DICTIONARY_H 
