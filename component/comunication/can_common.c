//
// Created by HuangShukun on 2024/3/2.
//

#include "can_common.h"
#include "memory.h"
#include "stdlib.h"
#include "CRC8_CRC16.h"
#include "can_hardwares.h"
/**
 * @brief 重置CAN comm的接收状态和buffer
 *
 * @param ins 需要重置的实例
 */
static void CANCommResetRx(CANCommInstance *ins)
{
    // 当前已经收到的buffer清零
    memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
    ins->recv_state = 0;   // 接收状态重置
    ins->cur_recv_len = 0; // 当前已经收到的长度重置
}

/**
 * @brief cancomm的接收回调函数
 *
 * @param _instance
 */
void CANCommRxCallback(CANCommInstance *_instance)
{
    /* 当前接收状态判断 */
    if (_instance->rx_buff[0] == CAN_COMM_HEADER && _instance->recv_state == 0) // 之前尚未开始接收且此次包里第一个位置是帧头
    {
        if (_instance->rx_buff[1] == _instance->recv_data_len) // 如果这一包里的datalen也等于我们设定接收长度(这是因为暂时不支持动态包长)
        {
            _instance->recv_state = 1; // 设置接收状态为1,说明已经开始接收
        }
        else
            return; // 直接跳过即可
    }

    if (_instance->recv_state) // 已经收到过帧头
    {
        // 如果已经接收到的长度加上当前一包的长度大于总buf len,说明接收错误
        if (_instance->cur_recv_len + _instance->rx_len > _instance->recv_buf_len)
        {
            CANCommResetRx(_instance);
            return; // 重置状态然后返回
        }

        // 直接把当前接收到的数据接到buffer后面
        memcpy(_instance->raw_recvbuf + _instance->cur_recv_len, _instance->rx_buff, _instance->rx_len);
        _instance->cur_recv_len += _instance->rx_len;

        // 收完这一包以后刚好等于总buf len,说明已经收完了
        if (_instance->cur_recv_len == _instance->recv_buf_len)
        {
            // 如果buff里本tail的位置等于CAN_COMM_TAIL
            if (_instance->raw_recvbuf[_instance->recv_buf_len - 1] == CAN_COMM_TAIL)
            { // 通过校验,复制数据到unpack_data中
                if (_instance->raw_recvbuf[_instance->recv_buf_len - 2] ==  get_CRC8_check_sum(_instance->raw_recvbuf + 2, _instance->recv_data_len, 0xff))
                { // 数据量大的话考虑使用DMA
                    memcpy(_instance->unpacked_recv_data, _instance->raw_recvbuf + 2, _instance->recv_data_len);
                    _instance->update_flag = 1;           // 数据更新flag置为1
                                                        //@todo: 这里可以加看门狗
                }
            }
            CANCommResetRx(_instance);
            return; // 重置状态然后返回
        }
    }
}

uint8_t CANTransmit(CANCommInstance *_instance, float timeout)
{
    float dwt_start = DWT_GetTimeline_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0) // 等待邮箱空闲
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
        {
            return 0;
        }
    }
    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox))
    {
        return 0;
    }
    return 1; // 发送成功


}
void CANSetDLC(CANCommInstance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
        while (1)
            ;
    _instance->txconf.DLC = length;
}
void CANCommSend(CANCommInstance *instance, uint8_t *data)
{
    static uint8_t crc8;
    static uint8_t send_len;
    // 将data copy到raw_sendbuf中,计算crc8
    memcpy(instance->raw_sendbuf + 2, data, instance->send_data_len);

    crc8 = get_CRC8_check_sum(data, instance->send_data_len, 0xff);
    instance->raw_sendbuf[2 + instance->send_data_len] = crc8;

    // CAN单次发送最大为8字节,如果超过8字节,需要分包发送
    for (size_t i = 0; i < instance->send_buf_len; i += 8)
    { // 如果是最后一包,send len将会小于8,要修改CAN的txconf中的DLC位
        send_len = instance->send_buf_len - i >= 8 ? 8 : instance->send_buf_len - i;
        CANSetDLC(instance, send_len);
        memcpy(instance->tx_buff, instance->raw_sendbuf + i, send_len);
        CANTransmit(instance, 1);
    }
}

void *CANCommGet(CANCommInstance *instance)
{
    instance->update_flag = 0; // 读取后将更新flag置为0
    return instance->unpacked_recv_data;
}

CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config)
{
    CANCommInstance *ins = (CANCommInstance *)malloc(sizeof(CANCommInstance));
    memset(ins, 0, sizeof(CANCommInstance));

    ins->recv_data_len = comm_config->recv_data_len;
    ins->recv_buf_len = comm_config->recv_data_len + CAN_COMM_OFFSET_BYTES; // head + datalen + crc8 + tail
    ins->send_data_len = comm_config->send_data_len;
    ins->send_buf_len = comm_config->send_data_len + CAN_COMM_OFFSET_BYTES;
    ins->raw_sendbuf[0] = CAN_COMM_HEADER;            // head,直接设置避免每次发送都要重新赋值,下面的tail同理
    ins->raw_sendbuf[1] = comm_config->send_data_len; // datalen
    ins->raw_sendbuf[comm_config->send_data_len + CAN_COMM_OFFSET_BYTES - 1] = CAN_COMM_TAIL;
    // 进行发送报文的配置
    ins->txconf.StdId = comm_config->tx_id; // 发送id
    ins->txconf.IDE = CAN_ID_STD;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    ins->txconf.RTR = CAN_RTR_DATA;    // 发送数据帧
    ins->txconf.DLC = 0x08;            // 默认发送长度为8
    // 设置回调函数和接收发送id
    ins->can_handle = comm_config->can_handle;
    ins->tx_id = comm_config->tx_id; // 接口，以后有用
    ins->rx_id = comm_config->rx_id;
    register_can_comm_instance(ins);
    return ins;
}