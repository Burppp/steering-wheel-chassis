////
//// Created by Jackson on 2023/11/7.
////
//
///* Include */
//#include "left_gimbal.h"
//#include "bsp_can.h"
//#include "Gimbal.h"
///*    define    */
//
///*    变量      */
//
//left_gimbal_t  leftgimbal;
//extern RC_ctrl_t  rc_ctrl;
//extern launcher_t launcher;
//extern fp32 INS_angle[3];
//extern fp32 INS_gyro[3];
//extern fp32 INS_quat[4];
//extern vision_t vision_data;
//extern robot_ctrl_info_t robot_ctrl;
//extern Referee_info_t Referee;
//extern receive_competition_info  competition;
//extern uint8_t critical_value;
//first_order_filter_type_t left_pitch_first_order_set;
//first_order_filter_type_t left_pitch_current_first_order_set;
//first_order_filter_type_t left_pitch_speed_in;
//first_order_filter_type_t left_yaw_first_order_set;
//first_order_filter_type_t left_yaw_speed;
//first_order_filter_type_t left_yaw_speed_in;
//first_order_filter_type_t left_yaw_angle;
//first_order_filter_type_t left_yaw_angle_in;
//second_lowPass_filter left_yaw_speed_out;
//second_lowPass_filter left_yaw_current_out;
//uint8_t left_gimbal_up_flag;
//uint8_t left_gimbal_down_flag;
//uint8_t left_gimbal_left_flag;
//uint8_t left_gimbal_right_flag;
//uint8_t left_gimbal_yaw_offset_flag;
//float32_t left_gimbal_yaw_spin_offset;
///* 函数及其声明  */
//static void left_gimbal_init();
//static void left_gimbal_mode_set();
//static void left_gimbal_back_handle();
//static void left_gimbal_active_handle();
//static void left_gimbal_relax_handle();
//static void left_gimbal_ctrl_loop_cal(int mode);
//static void left_gimbal_angle_update();
//static void left_gimbal_patrol_handle();
//static void left_gimbal_auto_handle();
//static void left_gimbal_mode_change();
//static void left_gimbal_can_send_back_mapping();
//
//_Noreturn void left_gimbal_task(void const*pvParameters){
//    //任务初始化时间
//    vTaskDelay(GIMBAL_TASK_INIT_TIME);
//
//    //云台初始化
//    left_gimbal_init();
//
//    //发射机构初始化
//    launcher_init();
//
//    //主任务体
//    while(1){
//
//        left_gimbal_angle_update();//更新绝对、相对角度接收值
//        //chassis_feedback_update();//在各自任务中发送会存在队列冲突问题，导致只有gimbal或odom的信息，因此放在一起发送
//        send_competition_info();
//
//        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误
//        left_gimbal_mode_set();//根据遥控器设置云台控制模式
//
//        launcher_mode_set();//发射模式设置
//
//        switch (leftgimbal.mode) {
//
//            case LEFT_GIMBAL_RELAX://云台失能
//                left_gimbal_relax_handle();
//                break;
//
//            case LEFT_GIMBAL_BACK://云台回中
//                left_gimbal_back_handle();
//                left_gimbal_ctrl_loop_cal(ABSOLUTE_CTRL);
//                break;
//
//            case LEFT_GIMBAL_ACTIVE://云台控制
//                left_gimbal_active_handle();  //得到遥控器对云台电机的控制
//                left_gimbal_ctrl_loop_cal(ABSOLUTE_CTRL);
//                break;
//
//            case LEFT_GIMBAL_PATROL://云台巡逻
//                left_gimbal_active_handle();  //得到遥控器对云台电机的控制
//                left_gimbal_patrol_handle();
//                break;
//            case LEFT_GIMBAL_AUTO:
//                left_gimbal_auto_handle();
//                left_gimbal_ctrl_loop_cal(ABSOLUTE_CTRL);
//                break;
//        }
//
//        launcher_control();//发射机构控制
//        if(leftgimbal.mode == GIMBAL_RELAX)
//        {
//            left_gimbal_relax_handle();
//        }
//        left_gimbal_can_send_back_mapping();
//
//        xTaskResumeAll();
//        vTaskDelay(1);
//    }
//}
//
//static void left_gimbal_init(){
//
//    leftgimbal.mode=leftgimbal.last_mode=GIMBAL_RELAX;//初始化默认状态为失能
//
//
//    //yaw轴电机 角度环和速度环PID初始化
//    pid_init(&leftgimbal.motor_gimbal[YAW].angle_p,
//             LEFT_GIMBAL_YAW_ANGLE_MAX_OUT,
//             LEFT_GIMBAL_YAW_ANGLE_MAX_IOUT,
//             LEFT_GIMBAL_YAW_ANGLE_PID_KP,
//             LEFT_GIMBAL_YAW_ANGLE_PID_KI,
//             LEFT_GIMBAL_YAW_ANGLE_PID_KD);
//
//    pid_init(&leftgimbal.motor_gimbal[YAW].relative_angle_p,
//             LEFT_GIMBAL_YAW_RELATIVE_ANGLE_MAX_OUT,
//             LEFT_GIMBAL_YAW_RELATIVE_ANGLE_MAX_IOUT,
//             LEFT_GIMBAL_YAW_RELATIVE_ANGLE_PID_KP,
//             LEFT_GIMBAL_YAW_RELATIVE_ANGLE_PID_KI,
//             LEFT_GIMBAL_YAW_RELATIVE_ANGLE_PID_KD);
//
//    pid_init(&leftgimbal.motor_gimbal[YAW].speed_p,
//             LEFT_GIMBAL_YAW_SPEED_MAX_OUT,
//             LEFT_GIMBAL_YAW_SPEED_MAX_IOUT,
//             LEFT_GIMBAL_YAW_SPEED_PID_KP,
//             LEFT_GIMBAL_YAW_SPEED_PID_KI,
//             LEFT_GIMBAL_YAW_SPEED_PID_KD);
//
//    pid_init(&leftgimbal.motor_gimbal[YAW].relative_speed_p,
//             LEFT_GIMBAL_YAW_RELATIVE_SPEED_MAX_OUT,
//             LEFT_GIMBAL_YAW_RELATIVE_SPEED_MAX_IOUT,
//             LEFT_GIMBAL_YAW_RELATIVE_SPEED_PID_KP,
//             LEFT_GIMBAL_YAW_RELATIVE_SPEED_PID_KI,
//             LEFT_GIMBAL_YAW_RELATIVE_SPEED_PID_KD);
//
//    //pit轴电机 角度环和速度环PID初始化
//    pid_init(&leftgimbal.motor_gimbal[PITCH].angle_p,
//             LEFT_GIMBAL_PITCH_ANGLE_MAX_OUT,
//             LEFT_GIMBAL_PITCH_ANGLE_MAX_IOUT,
//             LEFT_GIMBAL_PITCH_ANGLE_PID_KP,
//             LEFT_GIMBAL_PITCH_ANGLE_PID_KI,
//             LEFT_GIMBAL_PITCH_ANGLE_PID_KD);
//
//    pid_init(&leftgimbal.motor_gimbal[PITCH].speed_p,
//             LEFT_GIMBAL_PITCH_SPEED_MAX_OUT,
//             LEFT_GIMBAL_PITCH_SPEED_MAX_IOUT,
//             LEFT_GIMBAL_PITCH_SPEED_PID_KP,
//             LEFT_GIMBAL_PITCH_SPEED_PID_KI,
//             LEFT_GIMBAL_PITCH_SPEED_PID_KD);
//
//    //低通滤波初始化
//
//    first_order_filter_init(&left_pitch_first_order_set, 1, (float *)30);
//    first_order_filter_init(&left_pitch_current_first_order_set, 1, (float *)40);
//    first_order_filter_init(&left_pitch_speed_in, 1, (float *)30);
//    first_order_filter_init(&left_yaw_first_order_set, 5, (float *)40);
//    first_order_filter_init(&left_yaw_speed, 1, (float *)45);
//    first_order_filter_init(&left_yaw_speed_in, 1, (float *)40);
//    first_order_filter_init(&left_yaw_angle,1,(float *)40);
//    first_order_filter_init(&left_yaw_angle_in,1,(float *)40);
//
//    SetCutoffFreq(&left_yaw_current_out,500,131);
//    SetCutoffFreq(&left_yaw_speed_out,500,188 );
//
//    //初始化时 云台设为未回中状态
//    leftgimbal.left_yaw_is_back=0;
//    leftgimbal.left_pitch_is_back=0;
//    //上电时默认先设置成失能模式，再切换到当前遥控设置模式
//    leftgimbal.last_mode=LEFT_GIMBAL_RELAX;
//
//    //yaw轴和pitch轴电机的校准编码值
//    leftgimbal.motor_gimbal[YAW].motor_info.offset_ecd=0;//以上电初始化时yaw轴指向的xiangdui角为中值
//    leftgimbal.motor_gimbal[PITCH].motor_info.offset_ecd=0;
//    //等有车了 上monitor 测量
//    //设置云台巡逻时pitch轴初始状态
//    left_gimbal_up_flag=1;
//    left_gimbal_down_flag=0;
//    left_gimbal_left_flag=0;
//    left_gimbal_right_flag=1;
//    left_gimbal_yaw_offset_flag = 0;
//}
//
////云台模式设置（获取遥控器信息，判断模式）
//static void left_gimbal_mode_set(){
//    switch (rc_ctrl.rc.s[RC_s_R]) {
//
//        case RC_SW_DOWN:
//        {
//            leftgimbal.mode=LEFT_GIMBAL_RELAX;
//            leftgimbal.last_mode=LEFT_GIMBAL_RELAX;
//            break;
//        }
//
//        case RC_SW_MID:
//            leftgimbal.last_mode=leftgimbal.mode;
//            if(leftgimbal.last_mode==LEFT_GIMBAL_RELAX|| leftgimbal.last_mode==LEFT_GIMBAL_PATROL)
//            {
//                leftgimbal.mode=LEFT_GIMBAL_BACK;
//                leftgimbal.left_yaw_is_back=0;
//                leftgimbal.left_pitch_is_back=0;
//            }
//            else if(leftgimbal.left_yaw_is_back==1 && leftgimbal.left_pitch_is_back == 1)
//            {
//                leftgimbal.mode=LEFT_GIMBAL_ACTIVE;
//                left_gimbal_yaw_offset_flag = 0;
//            }
//            break;
//        case RC_SW_UP:
//        {
//            if(leftgimbal.left_yaw_is_back==1&&leftgimbal.left_pitch_is_back==1)
//            {
//                if(leftgimbal.last_mode==LEFT_GIMBAL_AUTO)//自瞄失效后默认回到一般模式，但是若上一个模式是巡逻则会进行计数，要到一定计数值才会回到巡逻模式，防止自瞄短暂失效后云台立刻移动开的问题
//                {
//                    switch_mode_time++;
//                    if(switch_mode_time>=100)
//                    {
//                        leftgimbal.last_mode = LEFT_GIMBAL_ACTIVE;
//                        switch_mode_time=0;
//                        leftgimbal.mode = LEFT_GIMBAL_PATROL;
//                    }
//                }
//                else if(leftgimbal.last_mode==LEFT_GIMBAL_ACTIVE)
//                {
//                    leftgimbal.mode = LEFT_GIMBAL_PATROL;
//                }
//            }
//            else{
//                leftgimbal.mode=LEFT_GIMBAL_BACK;
//            }
//        }
//            break;
//        default:{
//            break;
//        }
//
//    }
//    left_gimbal_mode_change();
//}
//
//static void left_gimbal_mode_change()
//{
//    if(leftgimbal.mode == LEFT_GIMBAL_ACTIVE)
//    {    //自瞄锁定 0x31表示自瞄数据无效
//        if(robot_ctrl.target_lock==0x31 && (detect_list[DETECT_AUTO_AIM].status==ONLINE))
//        {
//            leftgimbal.last_mode=LEFT_GIMBAL_ACTIVE;
//            leftgimbal.mode=LEFT_GIMBAL_AUTO;
//        }
//    }
//    else if(leftgimbal.mode == LEFT_GIMBAL_PATROL)
//    {   //自瞄判定
//        if(robot_ctrl.target_lock==0x31 && (detect_list[DETECT_AUTO_AIM].status==ONLINE))
//        {
//            leftgimbal.last_mode=LEFT_GIMBAL_PATROL;
//            leftgimbal.mode=LEFT_GIMBAL_AUTO;
//        }
//    }
//    else if(leftgimbal.mode==LEFT_GIMBAL_AUTO)
//    {   //自瞄失败判定
//        if((detect_list[DETECT_AUTO_AIM].status==ONLINE)||robot_ctrl.target_lock ==0x32)
//        {
//            leftgimbal.last_mode=LEFT_GIMBAL_AUTO;
//            leftgimbal.mode=LEFT_GIMBAL_ACTIVE;
//        }
//    }
//}
//
//
//static void left_gimbal_can_send_back_mapping() {
////    int16_t *real_motor_give_current[4];
////    real_motor_give_current[0] = &launcher.motor_launcher[FIRE_RIGHT].give_current;
////    real_motor_give_current[1] = &leftgimbal.motor_gimbal[PITCH].give_current;
////    real_motor_give_current[2] = &launcher.motor_launcher[FIRE_LEFT].give_current;
////    real_motor_give_current[3] = &launcher.motor_launcher[BARRLE].give_current;
////
////    can_send_dji_motor(CAN_1,
////                       CAN_DJI_MOTOR_0x1FF_ID,
////                       leftgimbal.motor_gimbal[YAW].give_current,
////                       launcher.motor_launcher[TRIGGER].give_current,
////                       0,
////                       0);
////
////    can_send_dji_motor(CAN_2,
////                       CAN_DJI_MOTOR_0x1FF_ID,
////                       *real_motor_give_current[0],
////                       *real_motor_give_current[1],
////                       *real_motor_give_current[2],
////                       *real_motor_give_current[3]);
//
//}
//
//static  void left_gimbal_back_handle()
//{
//    if(leftgimbal.left_yaw_is_back == 0)
//    {
//        leftgimbal.yaw.absolute_angle_set=leftgimbal.motor_gimbal[YAW].relative_angle_get;
//        leftgimbal.motor_gimbal[YAW].relative_angle_set=0;
//        leftgimbal.left_yaw_is_back=1;
//    }
//    //这里的-2是要测的！暂时写-2
//    if(leftgimbal.pitch.absolute_angle_get>=-2 && leftgimbal.pitch.absolute_angle_get<=2)
//    {
//        leftgimbal.left_pitch_is_back=1;
//    }
//    else{
//        leftgimbal.pitch.absolute_angle_set=0;
//    }
//
//}
//
//static void left_gimbal_active_handle()
//{   //注意！这里的set就是pid下面的输入，遥控器的数值。
//    leftgimbal.yaw.absolute_angle_set+=(float32_t)-rc_ctrl.rc.ch[YAW_CHANNEL]*RC_TO_YAW*GIMBAL_RC_MOVE_RATIO_YAW;
//    leftgimbal.pitch.absolute_angle_set+=(float32_t)rc_ctrl.rc.ch[PITCH_CHANNEL]*RC_TO_PITCH*GIMBAL_RC_MOVE_RATIO_PIT;
//
//    leftgimbal.pitch.absolute_angle_set= fp32_constrain(leftgimbal.pitch.absolute_angle_set,MIN_RELA_ANGLE,MAX_RELA_ANGLE);
//    //思考一下小云台的pitch限位要和大云台的pitch一样吗  rela是什么
//    //应该不一样的系数 小云台的pitch不能360旋转的，不然会撞，应该是除了另外一个云台的交界处，其它坐标系都可以
//    //然后yaw的限位也是在这里设置吗
//}
//
//static void left_gimbal_auto_handle()
//{
//    average_add(&MF_auto_pitch,robot_ctrl.pitch);
//    leftgimbal.yaw.absolute_angle_set=robot_ctrl.yaw;
//    leftgimbal.pitch.absolute_angle_set=MF_auto_pitch.aver_num;
//    //这里的abs是绝对值
//    leftgimbal.pitch.absolute_angle_set= fp32_constrain(leftgimbal.pitch.absolute_angle_set,MIN_ABS_ANGLE,MAX_ABS_ANGLE);
//}
//
//static void left_gimbal_relax_handle()
//{
//    leftgimbal.motor_gimbal[YAW].give_current=0;
//    leftgimbal.motor_gimbal[PITCH].give_current=0;
//    launcher_relax_handle();
//}
//
//static void left_gimbal_patrol_handle()
//{
//    if(HAL_GetTick()-nav_time>=1000)
//    {
//        if(robot_ctrl.left_patrol_angle!=robot_ctrl.right_patrol_angle)
//        {
//
//            if(left_gimbal_left_flag)
//            {
//                //为什么角度的获取和云台转动系数有关
//                leftgimbal.yaw.absolute_angle_set+=GIMBAL_YAW_PATROL_SPEED;
//                if(leftgimbal.yaw.absolute_angle_get>=(fp32)robot_ctrl.left_patrol_angle){
//                    left_gimbal_left_flag=0;
//                    left_gimbal_right_flag=1;
//                }
//            }
//                // 这里的robot_ctrl.right_patrol_angle是最大值的意思还是什么
//            else if(left_gimbal_right_flag)
//            {
//                leftgimbal.yaw.absolute_angle_set-=GIMBAL_YAW_PATROL_SPEED;
//                if(leftgimbal.yaw.absolute_angle_get<=(fp32)robot_ctrl.right_patrol_angle)
//                {
//                    left_gimbal_right_flag=0;
//                }
//            }
//            leftgimbal.pitch.absolute_angle_set=-2;
//            //要不要写pitch巡逻
//        }
//        else
//        {   //行为树发送的左右巡逻角度相同，进入360 记得有个角度限制
//            //等出车了 测一下都为0时会不会相撞
//            //一个云台180
//            leftgimbal.yaw.absolute_angle_set+=GIMBAL_YAW_PATROL_SPEED;
//            leftgimbal.pitch.absolute_angle_set=-2;
//            left_gimbal_ctrl_loop_cal(ABSOLUTE_CTRL);
//        }
//    }
//    else
//    {  //导航强制云台巡逻
//        leftgimbal.yaw.absolute_angle_set+=GIMBAL_YAW_PATROL_SPEED;
//        leftgimbal.pitch.absolute_angle_set=-2;
//        left_gimbal_ctrl_loop_cal(AUTO_CTRL);
//    }
//
//}
//
////云台电机闭环控制函数
////mode 0为手动控制yaw
////mode 1为导航控制yaw
//static void left_gimbal_ctrl_loop_cal(int mode)
//{
//
//    if(mode == ABSOLUTE_CTRL)//手动
//    {
//        //右云台绕圈进入绝对角循环 我只想让它0到180
//        //左云台0到-180
////        if(leftgimbal.yaw.absolute_angle_set>=180){
////            leftgimbal.yaw.absolute_angle_set-=180;
////        }
////        else if(leftgimbal.yaw.absolute_angle_set<=-180){
////            leftgimbal.yaw.absolute_angle_set+=360;
////        }
//        first_order_filter_cali(&left_yaw_angle_in,leftgimbal.yaw.absolute_angle_set);
//        first_order_filter_cali(&left_yaw_angle,leftgimbal.motor_gimbal[YAW].relative_angle_get);
//        leftgimbal.motor_gimbal[YAW].ecd_set= pid_loop_calc(&leftgimbal.motor_gimbal[YAW].angle_p,
//                                                             left_yaw_angle.out,
//                                                             left_yaw_angle_in.out,
//                                                             critical_value,
//                                                             -180+critical_value);
//        first_order_filter_cali(&left_yaw_speed,leftgimbal.motor_gimbal[YAW].motor_info.speed_rpm);
//        first_order_filter_cali(&left_yaw_speed_in,leftgimbal.motor_gimbal[YAW].ecd_set);
//        leftgimbal.motor_gimbal[YAW].give_current=(int16_t)-pid_calc(&leftgimbal.motor_gimbal[YAW].speed_p,
//                                                                     leftgimbal.motor_gimbal[YAW].motor_info.speed_rpm,
//                                                                     left_yaw_speed_in.out);
//    }
//    else if(mode == AUTO_CTRL)//强制
//    {
////        if(leftgimbal.yaw.absolute_angle_set>=180){
////            leftgimbal.yaw.absolute_angle_set-=360;
////        }
////        else if(leftgimbal.yaw.absolute_angle_set<=-180) {
////            leftgimbal.yaw.absolute_angle_set += 360;
////        }
//
//        first_order_filter_cali(&left_yaw_angle_in,leftgimbal.yaw.absolute_angle_set);
//        first_order_filter_cali(&left_yaw_angle,leftgimbal.motor_gimbal[YAW].relative_angle_get);
//        leftgimbal.motor_gimbal[YAW].ecd_set= pid_loop_calc(&leftgimbal.motor_gimbal[YAW].angle_p,
//                                                            left_yaw_angle.out,
//                                                            left_yaw_angle_in.out,
//                                                             critical_value,
//                                                             -180+critical_value);
//        first_order_filter_cali(&left_yaw_speed,leftgimbal.motor_gimbal[YAW].motor_info.speed_rpm);
//        first_order_filter_cali(&left_yaw_speed_in,leftgimbal.motor_gimbal[YAW].ecd_set);
//        leftgimbal.motor_gimbal[YAW].give_current=(int16_t)-pid_calc(&leftgimbal.motor_gimbal[YAW].speed_p,
//                                                                     leftgimbal.motor_gimbal[YAW].motor_info.speed_rpm,
//                                                                     left_yaw_speed_in.out);
//
//    }
//    leftgimbal.motor_gimbal[PITCH].ecd_set= pid_calc(&leftgimbal.motor_gimbal[PITCH].angle_p,
//                                                     leftgimbal.pitch.absolute_angle_get,
//                                                     leftgimbal.pitch.absolute_angle_set);
//    first_order_filter_cali(&left_pitch_speed_in,leftgimbal.motor_gimbal[PITCH].ecd_set);
//    leftgimbal.motor_gimbal[PITCH].relative_angle_get=((fp32)leftgimbal.motor_gimbal[PITCH].motor_info.speed_rpm*PI)/30.0f;
//    first_order_filter_cali(&left_yaw_first_order_set,leftgimbal.absolute_ecd_pitch);
//
//    leftgimbal.motor_gimbal[PITCH].give_current=(int16_t)-pid_calc(&leftgimbal.motor_gimbal[PITCH].speed_p,
//                                                                   left_pitch_current_first_order_set.out,
//                                                                   left_pitch_speed_in.out);
//    first_order_filter_cali(&left_pitch_first_order_set,leftgimbal.motor_gimbal[PITCH].give_current);
//    leftgimbal.motor_gimbal[PITCH].give_current=(int16_t)left_pitch_first_order_set.out;
//}
////云台角度更新
//static void left_gimbal_angle_update()
//{
//    leftgimbal.pitch.absolute_angle_get=motor_ecd_to_angle_change(leftgimbal.motor_gimbal[PITCH].motor_info.ecd,
//                                                                  leftgimbal.motor_gimbal[PITCH].motor_info.offset_ecd);
//    leftgimbal.motor_gimbal[PITCH].relative_angle_get= motor_ecd_to_angle_change(leftgimbal.motor_gimbal[PITCH].motor_info.ecd,
//                                                                                 leftgimbal.motor_gimbal[PITCH].motor_info.offset_ecd);
//
//    leftgimbal.yaw.absolute_angle_get=-motor_ecd_to_angle_change(leftgimbal.motor_gimbal[YAW].motor_info.ecd,
//                                                                 leftgimbal.motor_gimbal[YAW].motor_info.offset_ecd);
//    leftgimbal.motor_gimbal[YAW].relative_angle_get=-motor_ecd_to_angle_change(leftgimbal.motor_gimbal[YAW].motor_info.ecd,
//                                                                               leftgimbal.motor_gimbal[YAW].motor_info.offset_ecd);
//
//    leftgimbal.absolute_ecd_yaw=(fp32)INS_gyro[2];
//    leftgimbal.absolute_ecd_pitch=(fp32)INS_gyro[0];
//
//    vision_data.yaw=leftgimbal.yaw.absolute_angle_get;
//    vision_data.pitch=leftgimbal.pitch.absolute_angle_get;
//    vision_data.roll=(fp32)INS_angle[1]*MOTOR_RAD_TO_ANGLE;
//    for(int i=0;i<4;++i){
//        vision_data.quaternion[i]=INS_quat[i];
//    }
//    vision_data.id=Referee.GameRobotStat.robot_id;
//    vision_data.shoot_speed=Referee.ShootData.bullet_speed;
//    if(competition.enemy_outpost_hp<=0)
//        vision_data.shoot_sta = 1;
//    else vision_data.shoot_sta = 0;
//    rm_queue_data(VISION_ID,&vision_data,sizeof(vision_t));
//
//
//}