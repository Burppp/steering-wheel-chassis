#include "Referee.h"
#include "string.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "Gimbal.h"
#include "Detection.h"
#include "packet.h"
#include "protocol_shaob.h"

#include "can_common.h"
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern navigation_info nav_info;

Graphic_Operate static_update_flag=UI_ADD;
Graphic_Operate one_layer_update_flag=UI_ADD;
Graphic_Operate two_layer_update_flag=UI_ADD;
Graphic_Operate three_layer_update_flag=UI_ADD;

uint8_t usart6_buf[REFEREE_BUFFER_SIZE]={0};
uint32_t test_time;
Referee_info_t Referee;

//CANCommInstance* referee_can;
//CANComm_Init_Config_s referee_can_cfg = {
//        .tx_id = 0x203,
//        .send_data_len = sizeof(Referee_info_t),
//        .can_handle = &hcan2,
//};


/*
 * UI更新状态
 */
ui_robot_status_t ui_robot_status={

    .static_update=true,
    .gimbal_mode=GIMBAL_RELAX,

    .block_warning=false,
    .super_cap_value=0.f,
    .shoot_heat_limit=0,

};

/*函数和声明*/
static void referee_unpack_fifo_data(void);
static bool_t Referee_read_data(uint8_t *ReadFromUsart);
static void ui_static_draw();

/*裁判系统主任务*/

extern fp32 INS_angle[3];
extern DMA_HandleTypeDef hdma_usart1_tx;

//串口中断函数
void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);//读取UART6-SR 和UART6-DR; 清除中断标志位

        __HAL_DMA_DISABLE(huart6.hdmarx); //使能dma_rx

        Referee_read_data(&usart6_buf[0]);

        huart1.gState = HAL_UART_STATE_READY;
        hdma_usart1_tx.State = HAL_DMA_STATE_READY;
        __HAL_UNLOCK(&hdma_usart1_tx);
        usart1_tx_dma_enable((uint8_t*)&Referee, sizeof(Referee));
        uart_send_data((uint8_t*)&Referee, sizeof(Referee), REFEREE);
//        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&Referee, sizeof(Referee));
        huart1.gState = HAL_UART_STATE_READY;
        hdma_usart1_tx.State = HAL_DMA_STATE_READY;
        __HAL_UNLOCK(&hdma_usart1_tx);

//        memset(&usart6_buf[0],0,REFEREE_BUFFER_SIZE);//置0

        __HAL_DMA_CLEAR_FLAG(huart6.hdmarx,DMA_LISR_TCIF1); //清除传输完成标志位

        __HAL_DMA_SET_COUNTER(huart6.hdmarx, REFEREE_BUFFER_SIZE);//设置DMA 搬运数据大小 单位为字节

        __HAL_DMA_ENABLE(huart6.hdmarx); //使能DMARx

    }
}

//根据裁判系统信息判断机器人的ID和对应客户端的ID
void judge_team_client(){
    //本机器人为红方
    if(Referee.GameRobotStat.robot_id<10)
    {
        Referee.ids.teammate_hero 	   = 1;
        Referee.ids.teammate_engineer  = 2;
        Referee.ids.teammate_infantry3 = 3;
        Referee.ids.teammate_infantry4 = 4;
        Referee.ids.teammate_infantry5 = 5;
        Referee.ids.teammate_plane		 	= 6;
        Referee.ids.teammate_sentry		= 7;

        Referee.ids.client_hero 		 	= 0x0101;
        Referee.ids.client_engineer  = 0x0102;
        Referee.ids.client_infantry3 = 0x0103;
        Referee.ids.client_infantry4 = 0x0104;
        Referee.ids.client_infantry5 = 0x0105;
        Referee.ids.client_plane			= 0x0106;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_red:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;
            
            case Referee_engineer_red:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;
            
            case Referee_infantry3_red:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_red:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_red:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_red:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }//本机器人为蓝方
    else{
        Referee.ids.teammate_hero 		 	= 101;
        Referee.ids.teammate_engineer  = 102;
        Referee.ids.teammate_infantry3 = 103;
        Referee.ids.teammate_infantry4 = 104;
        Referee.ids.teammate_infantry5 = 105;
        Referee.ids.teammate_plane		 	= 106;
        Referee.ids.teammate_sentry		= 107;

        Referee.ids.client_hero 		 	= 0x0165;
        Referee.ids.client_engineer  = 0x0166;
        Referee.ids.client_infantry3 = 0x0167;
        Referee.ids.client_infantry4 = 0x0168;
        Referee.ids.client_infantry5 = 0x0169;
        Referee.ids.client_plane			= 0x016A;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_blue:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;

            case Referee_engineer_blue:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;

            case Referee_infantry3_blue:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_blue:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_blue:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_blue:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }
}

bool_t Referee_read_data(uint8_t *ReadFromUsart)
{
    int CmdID=0;//数据命令码解析

    uint16_t judge_length;
    Referee.RobotHurt.being_hurt = false;
    if(ReadFromUsart==NULL)
        return 0 ;

    memcpy(&Referee.FrameHeader,ReadFromUsart,Referee_LEN_FRAME_HEAD);

    if(ReadFromUsart[SOF]==REFREE_HEADER_SOF) //判断帧头是否为0xA5
    {
        if(verify_CRC8_check_sum(ReadFromUsart,LEN_HEADER)) //CRC
        {
            judge_length=ReadFromUsart[DATA_LENGTH]+LEN_HEADER+Referee_LEN_CMD_ID+Referee_LEN_FRAME_TAIL;
            if(verify_CRC16_check_sum(ReadFromUsart,judge_length))
            {
//                retval_tf=1;//表示数据可用
                CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)

                switch (CmdID)
                {

                    case Referee_ID_game_state://0x0001 比赛状态 1HZ
                        memcpy(&Referee.GameState,ReadFromUsart+DATA,Referee_LEN_game_state);
                        break;

                    case Referee_ID_game_result://0x0002 比赛结果   比赛结束后发送
                        memcpy(&Referee.GameResult,ReadFromUsart+DATA,Referee_LEN_game_result);
                        Referee.GameResult.game_over = true;
                        break;

                    case Referee_ID_game_robot_hp://0x0003 机器人状态HP   1HZ
                        memcpy(&Referee.GameRobotHP,ReadFromUsart+DATA,Referee_LEN_game_robot_survivors);
                        break;

//                    case Referee_ID_game_dart_state: //0x0004 飞镖发射状态
//                        memcpy(&Referee.GameDartStatus,ReadFromUsart+DATA,Referee_LED_game_missile_state);
//                        break;

//                    case Referee_ID_game_buff: //0x0005 ICRA_BUFF状态     1HZ
//                        memcpy(&Referee.GameICRABuff,ReadFromUsart+DATA,Referee_LED_game_buff);
//                        break;

                    case Referee_ID_event_data://0x0101 场地事件数据      1HZ
                        memcpy(&Referee.EventData,ReadFromUsart+DATA,Referee_LEN_event_data);
                        break;

                    case Referee_ID_supply_projectile_action://0x0102 场地补给站动作标识数据   动作改变之后发送
                        memcpy(&Referee.SupplyProjectileAction,ReadFromUsart+DATA,Referee_LEN_supply_projectile_action);
                        break;

                    case Referee_ID_supply_warm://0x0104    裁判系统警告数据    己方警告之后发送
                        memcpy(&Referee.RefereeWarning,ReadFromUsart+DATA,Referee_LEN_supply_warm);
                        break;

                    case Referee_ID_dart_data://0x0105    飞镖发射口倒计时    1HZ
                        memcpy(&Referee.DartRemainingTime,ReadFromUsart+DATA,Referee_LEN_missile_shoot_time);
                        break;

                    case Referee_ID_game_robot_system_data://0x0201   机器人状态数据     10HZ
                        memcpy(&Referee.GameRobotStat,ReadFromUsart+DATA,Referee_LEN_game_robot_state);
                        judge_team_client();//判断一下机器人所属的队伍和类型 以及对应的机械人id和客户端id
                        break;

                    case Referee_ID_power_heat_data://0x0202    实时功率热量数据    50HZ
                        memcpy(&Referee.PowerHeatData,ReadFromUsart+DATA,Referee_LEN_power_heat_data);
                        break;

                    case Referee_ID_game_robot_pos://0x0203     机器人位置数据     10HZ
                        memcpy(&Referee.GameRobotPos,ReadFromUsart+DATA,Referee_LEN_game_robot_pos);
                        break;

                    case Referee_ID_buff_musk://0x0204  机器人增益数据     1HZ
                        memcpy(&Referee.Buff,ReadFromUsart+DATA,Referee_LEN_buff_musk);
                        break;

                    case Referee_ID_aerial_robot_energy://0x0205    空中机器人能量状态数据 10HZ
                        memcpy(&Referee.AerialRobotEnergy,ReadFromUsart+DATA,Referee_LEN_aerial_robot_energy);
                        break;

                    case Referee_ID_robot_hurt://0x0206     伤害状态数据  伤害发生后发送
                        memcpy(&Referee.RobotHurt,ReadFromUsart+DATA,Referee_LEN_robot_hurt);
                        Referee.RobotHurt.being_hurt = true;//受击判断
                        break;

                    case Referee_ID_shoot_data://0x0207     实时射击数据  射击后发送
                        memcpy(&Referee.ShootData,ReadFromUsart+DATA,Referee_LEN_shoot_data);
                        break;

                    case Referee_ID_bullet_remaining://0x0208   剩余发射数   10HZ周期发送
                        memcpy(&Referee.BulletRemaining,ReadFromUsart+DATA,Referee_LEN_bullet_remaining);
                        break;

                    case Referee_ID_rfid_status://0x0209    机器人RFID状态，1Hz
                        memcpy(&Referee.RfidStatus,ReadFromUsart+DATA,Referee_LEN_rfid_status);
                        break;

                    case Referee_ID_dart_client_directive://0x020A  飞镖机器人客户端指令书, 10Hz
                        memcpy(&Referee.DartClient,ReadFromUsart+DATA,Referee_LEN_dart_client_directive);
                        break;

                    case Referee_ID_game_robot_pos_sentry://0x020B  机器人位置数据
                        memcpy(&Referee.GroundRobotPos, ReadFromUsart+DATA, Referee_LEN_robot_pos_sentry);
                        break;

                    case Referee_ID_radar_focusing_data://0x020C    雷达标记进度数据
                        memcpy(&Referee.RadarMark, ReadFromUsart+DATA, Referee_LEN_radar_focusing);
                        break;

                    case Referee_ID_sentry_decision_data://0x020D   哨兵自主决策信息同步
                        memcpy(&Referee.SentryMoveInfo, ReadFromUsart+DATA, Referee_LEN_sentry_decision);
                        break;

                    case Referee_ID_radar_decison_data://0x020E     雷达自主决策信息同步
                        memcpy(&Referee.RadarMoveInfo, ReadFromUsart+DATA, Referee_LEN_radar_decision);
                        break;

                    case Referee_ID_controller_interactive_header_data://0x0302 自定义控制器交互数据接口
//
                        break;

                    case Referee_ID_map_interactive_header_data://0x0303   客户端小地图交互数据
                        memcpy(&Referee.GoalInfo,ReadFromUsart+DATA,Referee_LEN_command_client_goal);
                        break;

                    case Referee_ID_keyboard_information://0x0304       键盘、鼠标信息
                        memcpy(&Referee.RemoteControl, ReadFromUsart+DATA, Referee_LEN_remote_controller);
                        break;

                    case Referee_ID_map_radar_data://0x0305             选手端小地图接收雷达数据
                        memcpy(&Referee.RadarMap, ReadFromUsart+DATA, Referee_LEN_map_radar_data);
                        break;

                    case Referee_ID_controller_interactive_data://0x0306    自定义控制器与选手端交互数据
                        memcpy(&Referee.ClientControl, ReadFromUsart+DATA, Referee_LEN_controller_interactive);
                        break;

                    case Referee_ID_map_sentry_data://0x0307                选手端小地图接收哨兵数据
                        memcpy(&Referee.SentryMovePath, ReadFromUsart+DATA, Referee_LEN_map_sentry_data);
                        break;

                    case Referee_ID_map_robot_data://0x0308                 选手端小地图接收机器人数据
                        memcpy(&Referee.RobotCommunication, ReadFromUsart+DATA, Referee_LEN_map_robot_data);
                        break;

                    case Referee_ID_robot_interactive_header_data: //0x0301 测试机器人交互
                        if(Referee.radar_message.header.receiver_ID == 7||Referee.radar_message.header.receiver_ID==107)
                            memcpy(&Referee.radar_message,ReadFromUsart+DATA, sizeof(radar_message));
                        break;
                    default:
                        break;
                }
            }
        }
        if(*(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length +Referee_LEN_FRAME_TAIL) == 0xA5)
        {
            //如果一个数据包出现了多帧数据,则再次读取
            Referee_read_data(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length+ Referee_LEN_FRAME_TAIL);
        }
    }
}


uint8_t ClientTxBuffer[200];//发送给客户端的数据缓冲区
static ext_map_sentry_data_t sentryDate;//发送变量
void send_nav_info()
{
    //裁判通信帧头处理
    sentryDate.txFrameHeader.SOF=REFREE_HEADER_SOF;
    sentryDate.txFrameHeader.data_length = sizeof(map_sentry_data_t);
    sentryDate.txFrameHeader.seq=0;
    memcpy(ClientTxBuffer,&sentryDate.txFrameHeader,sizeof(frame_header_struct_t));
    //CRC8
    append_CRC8_check_sum(ClientTxBuffer,sizeof(frame_header_struct_t));

    sentryDate.CmdID= Referee_ID_map_sentry_data;

    sentryDate.clientData.intention = nav_info.purpose;
    sentryDate.clientData.start_position_x = nav_info.start_point_x;
    sentryDate.clientData.start_position_y = nav_info.start_point_y;
    for (int i = 0; i < 49; ++i) {
        sentryDate.clientData.delta_x[i] = nav_info.delta_x[i];
        sentryDate.clientData.delta_y[i] = nav_info.delta_y[i];
    }
    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD,(uint8_t*)&sentryDate.CmdID,Referee_LEN_CMD_ID + sentryDate.txFrameHeader.data_length);
    append_CRC16_check_sum(ClientTxBuffer,sizeof(sentryDate));
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(sentryDate));
}
receive_competition_info competition;
bool our_outpost_destroyed = false;
void send_competition_info()//send to nuc
{
    competition.goal_point_x = Referee.GoalInfo.target_position_x;
    competition.goal_point_y = Referee.GoalInfo.target_position_y;
    competition.goal_point_z = Referee.GoalInfo.target_position_z;
    competition.command_info = Referee.GoalInfo.commd_keyboard;
    competition.game_state = Referee.GameState.game_progress;//0 is not start game , 4 is start game
    if(Referee.GameRobotStat.robot_id==7){ //7 means I'm red , 107 means I'm blue
        competition.our_outpost_hp = Referee.GameRobotHP.red_outpost_HP;
        competition.enemy_outpost_hp = Referee.GameRobotHP.blue_outpost_HP;
    } else if(Referee.GameRobotStat.robot_id == 107){
        competition.our_outpost_hp = Referee.GameRobotHP.blue_outpost_HP;
        competition.enemy_outpost_hp = Referee.GameRobotHP.red_outpost_HP;
    }
    //行为树测试代码 begin
//    competition.game_state = 4;
//            if((HAL_GetTick() - test_time)<60000) //7min ~ 6:20,10s~1min
//            {
//                competition.our_outpost_hp = 1500; //骚扰对面英雄时间
//                competition.enemy_outpost_hp = 1500;
//            }
//            else if((HAL_GetTick() - test_time)<180000) // 6:20~5:00,1min~3min
//            {
//                competition.our_outpost_hp = 1500; //多点随机巡逻并跟随老哥时间
//                competition.enemy_outpost_hp = 1500;
//            }
//            else if((HAL_GetTick() - test_time)<240000)//5:00~4:00,3min~4min
//            {
//                competition.our_outpost_hp = 1000;//对方前哨站勾八
//                competition.enemy_outpost_hp = 0;
//            }
//            else if((HAL_GetTick() - test_time)<300000)//4:00~3:00,4min~5min
//            {
//                competition.our_outpost_hp = 800;//我方前哨站低于1000 血
//                competition.enemy_outpost_hp = 0;
//            }
//            else if((HAL_GetTick() - test_time)<360000)//3:00~1:00,5min~6min
//            {
//                competition.our_outpost_hp = 200;//我方前哨站勾八
//                competition.enemy_outpost_hp = 0;
//            }
//            else if((HAL_GetTick() - test_time)<420000)//1:00~0:00
//            {
//                competition.our_outpost_hp = 0;//都勾八
//                competition.enemy_outpost_hp = 0;
//            }
    //行为树测试代码 end
    competition.remain_bullet = Referee.BulletRemaining.bullet_remaining_num_17mm;
    rm_queue_data(RECEIVE_COMPETITION_INFO_CMD_ID, &competition, sizeof(receive_competition_info));
}

void Referee_send_task(void const*argument){
    vTaskDelay(20);
    test_time = HAL_GetTick();
//    referee_can = CANCommInit(&referee_can_cfg);
    while (1)
    {
        send_nav_info();
        //当前哨站勾八时强制陀螺
        if(competition.our_outpost_hp<100&&Referee.GameState.game_progress==4) {
            our_outpost_destroyed = true;
        }
        else our_outpost_destroyed = false;
//        CANCommSend(referee_can, (uint8_t*)&Referee);
//
//        huart1.gState = HAL_UART_STATE_READY;
//        hdma_usart1_tx.State = HAL_DMA_STATE_READY;
//        __HAL_UNLOCK(&hdma_usart1_tx);
//        usart1_tx_dma_enable((uint8_t*)&Referee, sizeof(Referee));
//        uart_send_data((uint8_t*)&Referee, sizeof(Referee), REFEREE);
////        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&Referee, sizeof(Referee));
//        huart1.gState = HAL_UART_STATE_READY;
//        hdma_usart1_tx.State = HAL_DMA_STATE_READY;
//        __HAL_UNLOCK(&hdma_usart1_tx);

        vTaskDelay(30);//20ms 底盘功率发送频率是50HZ
    }
}