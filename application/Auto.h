////
//// Created by xhuanc on 2022/1/19.
////
//
//#ifndef _AUTO_H_
//#define _AUTO_H_
//
//#include "struct_typedef.h"
//
//#define AUTO_TASK_INIT_TIME 7200
//#define HEAD_LEN 2 //帧头长度
//#define FRAME_LEN 33 //一帧长度
//#define VISION_BUFFER_SIZE 200 //字节缓冲区长度
//#define VISION_BUFFER_SEND 20
//typedef struct {
//    uint8_t head;
//    uint8_t cmd;
////    uint8_t crc8;
//}vision_frame;
//typedef union{
//    uint8_t data[4];
//    fp32 value;
//}fp32_8;
//
//typedef union{
//    uint8_t data[2];
//    int16_t value;
//}int16_8;
//
//typedef struct {
//
//    vision_frame frame_header;
//    union {
//        uint8_t data[4];
//        fp32 value;
//    }pitch;
//
//    union {
//        uint8_t data[4];
//        fp32 value;
//    }yaw;
//
//}Vision_info_get;
//
//typedef struct {
//
//    vision_frame frame_header;
//    int16_8 yaw;
//    int16_8 pitch;
//    int16_8 roll;
//    int16_8 shoot_speed;
//
//
//}Vision_info_send;
//
//extern uint8_t usart1_receive_buf[VISION_BUFFER_SIZE];
//extern Vision_info_send Vision_send;
//extern Vision_info_get Vision_info;
//extern void auto_task(void const *pvParameters);
//
//
//#endif
