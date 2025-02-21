////
//// Created by xhuanc on 2022/1/19.
////
//
//#include "Auto.h"
//#include "string.h"
//#include "CRC8_CRC16.h"
//#include "cmsis_os.h"
//#include "bsp_usart.h"
//#include "string.h"
//#include "Referee.h"
//#include "Detection.h"
//#include "filter.h"
//#include "Chassis.h"
//extern UART_HandleTypeDef huart1;
//extern chassis_t  chassis;
//uint8_t usart1_receive_buf[VISION_BUFFER_SIZE]={0};
//uint8_t usart1_send_buf[VISION_BUFFER_SEND]={0};
//Vision_info_get Vision_info;
//Vision_info_send Vision_send;
//
//
//bool_t Vision_read_data(uint8_t *ReadFromUsart)
//{
//    uint16_t data_len;
//    if(ReadFromUsart==NULL)
//        return false;
//
//    if(ReadFromUsart[0]==0XA5)//帧头校验
//    {
//        if (verify_CRC8_check_sum(&ReadFromUsart[1], 5)) {
//            Vision_info.yaw.data[0] = ReadFromUsart[1];
//            Vision_info.yaw.data[1] = ReadFromUsart[2];
//            Vision_info.yaw.data[2] = ReadFromUsart[3];
//            Vision_info.yaw.data[3] = ReadFromUsart[4];
//        }
//
//        if (verify_CRC8_check_sum(&ReadFromUsart[6], 5)) {
//            Vision_info.pitch.data[0] = ReadFromUsart[6];
//            Vision_info.pitch.data[1] = ReadFromUsart[7];
//            Vision_info.pitch.data[2] = ReadFromUsart[8];
//            Vision_info.pitch.data[3] = ReadFromUsart[9];
//        }
//        Vision_info.frame_header.cmd = ReadFromUsart[11];
//
//        if(verify_CRC8_check_sum(&ReadFromUsart[12],5))
//        {
//            chassis.auto_vx.data[0] = ReadFromUsart[12];
//            chassis.auto_vx.data[1] = ReadFromUsart[13];
//            chassis.auto_vx.data[2] = ReadFromUsart[14];
//            chassis.auto_vx.data[3] = ReadFromUsart[15];
//        }
//
//        if(verify_CRC8_check_sum(&ReadFromUsart[17],5))
//        {
//            chassis.auto_vy.data[0] = ReadFromUsart[17];
//            chassis.auto_vy.data[1] = ReadFromUsart[18];
//            chassis.auto_vy.data[2] = ReadFromUsart[19];
//            chassis.auto_vy.data[3] = ReadFromUsart[20];
//        }
//
//        if(verify_CRC8_check_sum(&ReadFromUsart[22],5))
//        {
//            chassis.auto_vw.data[0] = ReadFromUsart[22];
//            chassis.auto_vw.data[1] = ReadFromUsart[23];
//            chassis.auto_vw.data[2] = ReadFromUsart[24];
//            chassis.auto_vw.data[3] = ReadFromUsart[25];
//        }
//        detect_handle(DETECT_AUTO_AIM);
//    }
//    //如果一个数据包出现了多帧数据,则再次读取
////    if(*(ReadFromUsart+26) == 0xA5)
////    {
////        Vision_read_data(ReadFromUsart+26);
////    }
//    return true;
//}
//
//void USART1_IRQHandler(void)
//{
//    static volatile uint8_t res;
//    if(USART1->SR & UART_FLAG_IDLE)
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart1);//读取UART1-SR 和UART1-DR; 清除中断标志位
//
//        __HAL_DMA_DISABLE(huart1.hdmarx); //失能dma_rx
//
//        Vision_read_data(&usart1_receive_buf[0]);//解析数据信息
//
//        memset(&usart1_receive_buf[0],0,VISION_BUFFER_SIZE);//置0
//
//        __HAL_DMA_CLEAR_FLAG(huart1.hdmarx,DMA_HISR_TCIF5); //清除传输完成标志位
//
//        __HAL_DMA_SET_COUNTER(huart1.hdmarx, VISION_BUFFER_SIZE);//设置DMA 搬运数据大小 单位为字节
//
//        __HAL_DMA_ENABLE(huart1.hdmarx); //使能DMAR
//
//    }
//
//
//}
//static void auto_init(){
//    //接收结构体
//    Vision_info.frame_header.head=0xA5;
//    Vision_info.yaw.value=0;
//    Vision_info.pitch.value=0;
//
//    //发送结构体
//    Vision_send.frame_header.head=0xA5;
//    Vision_send.yaw.value=0;
//    Vision_send.pitch.value=0;
//    Vision_send.roll.value=0;
//    Vision_send.shoot_speed.value=0;
//
//}
//static void vision_send(){
//
//    usart1_send_buf[0]= Vision_send.frame_header.head;
//
//    usart1_send_buf[1]=Vision_send.yaw.data[0];
//    usart1_send_buf[2]=Vision_send.yaw.data[1];
//    append_CRC8_check_sum(&usart1_send_buf[1],3);//3
//
//    usart1_send_buf[4]=Vision_send.pitch.data[0];
//    usart1_send_buf[5]=Vision_send.pitch.data[1];
//    append_CRC8_check_sum(&usart1_send_buf[6],3);//6
//
//    usart1_send_buf[7]=Vision_send.roll.data[0];
//    usart1_send_buf[8]=Vision_send.roll.data[1];
//    append_CRC8_check_sum(&usart1_send_buf[11],3);//9
//
//    //test
////    chassis.x_vel.value = 100;
////    chassis.y_vel.value = 0;
////    chassis.z_vel.value = 0;
//    usart1_send_buf[10]=chassis.x_vel.data[0];
//    usart1_send_buf[11]=chassis.x_vel.data[1];
//    append_CRC8_check_sum(&usart1_send_buf[10],3);//12
//
//    usart1_send_buf[13]=chassis.y_vel.data[0];
//    usart1_send_buf[14]=chassis.y_vel.data[1];
//    append_CRC8_check_sum(&usart1_send_buf[13],3);//15
//
//    usart1_send_buf[16]=chassis.z_vel.data[0];
//    usart1_send_buf[17]=chassis.z_vel.data[1];
//    append_CRC8_check_sum(&usart1_send_buf[16],3);//18
//
//    if(Referee.GameRobotStat.robot_id<10)//红色方的ID小于10
//    {
//        usart1_send_buf[19]=1;
//    }
//    else{
//        usart1_send_buf[19]=0;
//    }
//
//}
//
//void auto_task(void const *pvParameters)
//{
//    osDelay(AUTO_TASK_INIT_TIME);
//    auto_init();
//    append_CRC8_check_sum(usart1_send_buf,HEAD_LEN);
//    while (1)
//    {
////        if(Referee.ShootData.shooter_id==1)
////        {
//        Vision_send.shoot_speed.value=27.f;
////        }
//        //装载视觉发送信息          ---> 将Vision_info_send 的装进uart缓冲区
//        vision_send();
//        usart1_tx_dma_enable(usart1_send_buf,VISION_BUFFER_SEND);
//        osDelay(2);
//    }
//
//}