//
// Created by HuangShukun on 2024/3/2.
//

#ifndef REMOTE_CUBE_SENTRY_CAN_COMMON_H
#define REMOTE_CUBE_SENTRY_CAN_COMMON_H
#include "can.h"
#include "bsp_dwt.h"

#define CAN_COMM_MAX_BUFFSIZE 60 // 最大发送/接收字节数,如果不够可以增加此数值
#define CAN_COMM_HEADER 's'      // 帧头
#define CAN_COMM_TAIL 'e'        // 帧尾
#define CAN_COMM_OFFSET_BYTES 4  // 's'+ datalen + 'e' + crc8

#pragma pack(1)
typedef struct
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8

    /* 发送部分 */
    uint8_t send_data_len; // 发送数据长度
    uint8_t send_buf_len;  // 发送缓冲区长度,为发送数据长度+帧头单包数据长度帧尾以及校验和(4)
    uint8_t raw_sendbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES]; // 额外4个bytes保存帧头帧尾和校验和
    /* 接收部分 */
    uint8_t recv_data_len; // 接收数据长度
    uint8_t recv_buf_len;  // 接收缓冲区长度,为接收数据长度+帧头单包数据长度帧尾以及校验和(4)
    uint8_t raw_recvbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES]; // 额外4个bytes保存帧头帧尾和校验和
    uint8_t unpacked_recv_data[CAN_COMM_MAX_BUFFSIZE];                  // 解包后的数据,调用CANCommGet()后cast成对应的类型通过指针读取即可
    /* 接收和更新标志位*/
    uint8_t recv_state;   // 接收状态,
    uint8_t cur_recv_len; // 当前已经接收到的数据长度(包括帧头帧尾datalen和校验和)
    uint8_t update_flag;  // 数据更新标志位,当接收到新数据时,会将此标志位置1,调用CANCommGet()后会将此标志位置0

} CANCommInstance;
#pragma pack()
typedef struct
{
    CAN_HandleTypeDef *can_handle;              // can句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    uint8_t send_data_len;        // 发送数据长度
    uint8_t recv_data_len;        // 接收数据长度

} CANComm_Init_Config_s;
/**
 * @brief 初始化CANComm
 *
 * @param config CANComm初始化结构体
 * @return CANCommInstance*
 */
CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config);
/**
 * @brief cancomm的接收回调函数
 *
 * @param _instance
 */
void CANCommRxCallback(CANCommInstance *_instance);
/**
 * @brief 通过CANComm发送数据
 *
 * @param instance cancomm实例
 * @param data 注意此地址的有效数据长度需要和初始化时传入的datalen相同
 */
void CANCommSend(CANCommInstance *instance, uint8_t *data);
/**
 * @brief 通过CANComm获取数据
 *
 * @param instance cancomm实例
 * @return void* 返回数据指针
 */
void *CANCommGet(CANCommInstance *instance);
#endif //REMOTE_CUBE_SENTRY_CAN_COMMON_H
