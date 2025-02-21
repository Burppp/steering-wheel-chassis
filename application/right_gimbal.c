////
//// Created by Jackson on 2023/11/7.
////
//
///* Include */
//#include "right_gimbal.h"
//#include "bsp_can.h"
//#include "Gimbal.h"
///*    define    */
//
///*    变量      */
//
//right_gimbal_t  rightgimbal;
//extern RC_ctrl_t  rc_ctrl;
//extern launcher_t launcher;
//extern fp32 INS_angle[3];
//extern fp32 INS_gyro[3];
//extern fp32 INS_quat[4];
//extern vision_t vision_data;
//extern robot_ctrl_info_t robot_ctrl;
//extern Referee_info_t Referee;
//extern receive_competition_info  competition;
//first_order_filter_type_t right_pitch_first_order_set;
//first_order_filter_type_t right_pitch_current_first_order_set;
//first_order_filter_type_t right_pitch_speed_in;
//first_order_filter_type_t right_yaw_first_order_set;
//first_order_filter_type_t right_yaw_speed;
//first_order_filter_type_t right_yaw_speed_in;
//first_order_filter_type_t right_yaw_angle;
//first_order_filter_type_t right_yaw_angle_in;
//second_lowPass_filter right_yaw_speed_out;
//second_lowPass_filter right_yaw_current_out;
//uint8_t right_gimbal_up_flag;
//uint8_t right_gimbal_down_flag;
//uint8_t right_gimbal_left_flag;
//uint8_t right_gimbal_right_flag;
//uint8_t right_gimbal_yaw_offset_flag;
//uint8_t critical_value =5*MOTOR_ECD_TO_ANGLE;//左云台和右云台之间的临界的角度
//float32_t right_gimbal_yaw_spin_offset;
///* 函数及其声明  */
//static void right_gimbal_init();
//static void right_gimbal_mode_set();
//static void right_gimbal_back_handle();
//static void right_gimbal_active_handle();
//static void right_gimbal_relax_handle();
//static void right_gimbal_ctrl_loop_cal(int mode);
//static void right_gimbal_angle_update();
//static void right_gimbal_patrol_handle();
//static void right_gimbal_auto_handle();
//static void right_gimbal_mode_change();
//static void right_gimbal_can_send_back_mapping();
//
//_Noreturn void right_gimbal_task(void const*pvParameters){
//    //任务初始化时间
//    vTaskDelay(GIMBAL_TASK_INIT_TIME);
//
//    //云台初始化
//    right_gimbal_init();
//
//    //发射机构初始化
//    launcher_init();
//
//    //主任务体
//    while(1){
//
//        right_gimbal_angle_update();//更新绝对、相对角度接收值
//        //chassis_feedback_update();//在各自任务中发送会存在队列冲突问题，导致只有gimbal或odom的信息，因此放在一起发送
//        send_competition_info();
//
//        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误
//        right_gimbal_mode_set();//根据遥控器设置云台控制模式
//
//        launcher_mode_set();//发射模式设置
//
//        switch (rightgimbal.mode) {
//
//            case RIGHT_GIMBAL_RELAX://云台失能
//                right_gimbal_relax_handle();
//                break;
//
//            case RIGHT_GIMBAL_BACK://云台回中
//                right_gimbal_back_handle();
//                right_gimbal_ctrl_loop_cal(ABSOLUTE_CTRL);
//                break;
//
//            case RIGHT_GIMBAL_ACTIVE://云台控制
//                right_gimbal_active_handle();  //得到遥控器对云台电机的控制
//                right_gimbal_ctrl_loop_cal(ABSOLUTE_CTRL);
//                break;
//
//            case RIGHT_GIMBAL_PATROL://云台巡逻
//                right_gimbal_active_handle();  //得到遥控器对云台电机的控制
//                right_gimbal_patrol_handle();
//                break;
//            case RIGHT_GIMBAL_AUTO:
//                right_gimbal_auto_handle();
//                right_gimbal_ctrl_loop_cal(ABSOLUTE_CTRL);
//                break;
//        }
//
//        launcher_control();//发射机构控制
//        if(rightgimbal.mode == GIMBAL_RELAX)
//        {
//            right_gimbal_relax_handle();
//        }
//        right_gimbal_can_send_back_mapping();
//
//        xTaskResumeAll();
//        vTaskDelay(1);
//    }
//}
//
//static void right_gimbal_init(){
//
//    rightgimbal.mode=rightgimbal.last_mode=GIMBAL_RELAX;//初始化默认状态为失能
//
//
//    //yaw轴电机 角度环和速度环PID初始化
//    pid_init(&rightgimbal.motor_gimbal[YAW].angle_p,
//             RIGHT_GIMBAL_YAW_ANGLE_MAX_OUT,
//             RIGHT_GIMBAL_YAW_ANGLE_MAX_IOUT,
//             RIGHT_GIMBAL_YAW_ANGLE_PID_KP,
//             RIGHT_GIMBAL_YAW_ANGLE_PID_KI,
//             RIGHT_GIMBAL_YAW_ANGLE_PID_KD);
//
//    pid_init(&rightgimbal.motor_gimbal[YAW].relative_angle_p,
//             RIGHT_GIMBAL_YAW_RELATIVE_ANGLE_MAX_OUT,
//             RIGHT_GIMBAL_YAW_RELATIVE_ANGLE_MAX_IOUT,
//             RIGHT_GIMBAL_YAW_RELATIVE_ANGLE_PID_KP,
//             RIGHT_GIMBAL_YAW_RELATIVE_ANGLE_PID_KI,
//             RIGHT_GIMBAL_YAW_RELATIVE_ANGLE_PID_KD);
//
//    pid_init(&rightgimbal.motor_gimbal[YAW].speed_p,
//             RIGHT_GIMBAL_YAW_SPEED_MAX_OUT,
//             RIGHT_GIMBAL_YAW_SPEED_MAX_IOUT,
//             RIGHT_GIMBAL_YAW_SPEED_PID_KP,
//             RIGHT_GIMBAL_YAW_SPEED_PID_KI,
//             RIGHT_GIMBAL_YAW_SPEED_PID_KD);
//
//    pid_init(&rightgimbal.motor_gimbal[YAW].relative_speed_p,
//             RIGHT_GIMBAL_YAW_RELATIVE_SPEED_MAX_OUT,
//             RIGHT_GIMBAL_YAW_RELATIVE_SPEED_MAX_IOUT,
//             RIGHT_GIMBAL_YAW_RELATIVE_SPEED_PID_KP,
//             RIGHT_GIMBAL_YAW_RELATIVE_SPEED_PID_KI,
//             RIGHT_GIMBAL_YAW_RELATIVE_SPEED_PID_KD);
//
//    //pit轴电机 角度环和速度环PID初始化
//    pid_init(&rightgimbal.motor_gimbal[PITCH].angle_p,
//             RIGHT_GIMBAL_PITCH_ANGLE_MAX_OUT,
//             RIGHT_GIMBAL_PITCH_ANGLE_MAX_IOUT,
//             RIGHT_GIMBAL_PITCH_ANGLE_PID_KP,
//             RIGHT_GIMBAL_PITCH_ANGLE_PID_KI,
//             RIGHT_GIMBAL_PITCH_ANGLE_PID_KD);
//
//    pid_init(&rightgimbal.motor_gimbal[PITCH].speed_p,
//             RIGHT_GIMBAL_PITCH_SPEED_MAX_OUT,
//             RIGHT_GIMBAL_PITCH_SPEED_MAX_IOUT,
//             RIGHT_GIMBAL_PITCH_SPEED_PID_KP,
//             RIGHT_GIMBAL_PITCH_SPEED_PID_KI,
//             RIGHT_GIMBAL_PITCH_SPEED_PID_KD);
//
//    //低通滤波初始化
//
//    first_order_filter_init(&right_pitch_first_order_set, 1, (float *)30);
//    first_order_filter_init(&right_pitch_current_first_order_set, 1, (float *)40);
//    first_order_filter_init(&right_pitch_speed_in, 1, (float *)30);
//    first_order_filter_init(&right_yaw_first_order_set, 5, (float *)40);
//    first_order_filter_init(&right_yaw_speed, 1, (float *)45);
//    first_order_filter_init(&right_yaw_speed_in, 1, (float *)40);
//    first_order_filter_init(&right_yaw_angle,1,(float *)40);
//    first_order_filter_init(&right_yaw_angle_in,1,(float *)40);
//
//    SetCutoffFreq(&right_yaw_current_out,500,131);
//    SetCutoffFreq(&right_yaw_speed_out,500,188 );
//
//    //初始化时 云台设为未回中状态
//    rightgimbal.right_yaw_is_back=0;
//    rightgimbal.right_pitch_is_back=0;
//    //上电时默认先设置成失能模式，再切换到当前遥控设置模式
//    rightgimbal.last_mode=RIGHT_GIMBAL_RELAX;
//
//    //yaw轴和pitch轴电机的校准编码值
//    rightgimbal.motor_gimbal[YAW].motor_info.offset_ecd=0;//以上电初始化时yaw轴指向的xiangdui角为中值
//    rightgimbal.motor_gimbal[PITCH].motor_info.offset_ecd=0;
//    //等有车了 上monitor 测量
//    //设置云台巡逻时pitch轴初始状态
//    right_gimbal_up_flag=1;
//    right_gimbal_down_flag=0;
//    right_gimbal_left_flag=0;
//    right_gimbal_right_flag=1;
//    right_gimbal_yaw_offset_flag = 0;
//}
//
////云台模式设置（获取遥控器信息，判断模式）
//static void right_gimbal_mode_set(){
//    switch (rc_ctrl.rc.s[RC_s_R]) {
//
//        case RC_SW_DOWN:
//        {
//            rightgimbal.mode=RIGHT_GIMBAL_RELAX;
//            rightgimbal.last_mode=RIGHT_GIMBAL_RELAX;
//            break;
//        }
//
//        case RC_SW_MID:
//            rightgimbal.last_mode=rightgimbal.mode;
//            if(rightgimbal.last_mode==RIGHT_GIMBAL_RELAX || rightgimbal.last_mode==RIGHT_GIMBAL_PATROL)
//            {
//                rightgimbal.mode=RIGHT_GIMBAL_BACK;
//                rightgimbal.right_yaw_is_back=0;
//                rightgimbal.right_pitch_is_back=0;
//            }
//            else if(rightgimbal.right_yaw_is_back==1 && rightgimbal.right_pitch_is_back == 1)
//            {
//                rightgimbal.mode=RIGHT_GIMBAL_ACTIVE;
//                right_gimbal_yaw_offset_flag = 0;
//            }
//            break;
//        case RC_SW_UP:
//        {
//            if(rightgimbal.right_yaw_is_back==1&&rightgimbal.right_pitch_is_back==1)
//            {
//                if(rightgimbal.last_mode==RIGHT_GIMBAL_AUTO)//自瞄失效后默认回到一般模式，但是若上一个模式是巡逻则会进行计数，要到一定计数值才会回到巡逻模式，防止自瞄短暂失效后云台立刻移动开的问题
//                {
//                    switch_mode_time++;
//                    if(switch_mode_time>=100)
//                    {
//                        rightgimbal.last_mode = RIGHT_GIMBAL_ACTIVE;
//                        switch_mode_time=0;
//                        rightgimbal.mode = RIGHT_GIMBAL_PATROL;
//                    }
//                }
//                else if(rightgimbal.last_mode==RIGHT_GIMBAL_ACTIVE)
//                {
//                    rightgimbal.mode = RIGHT_GIMBAL_PATROL;
//                }
//            }
//            else{
//                rightgimbal.mode=RIGHT_GIMBAL_BACK;
//            }
//        }
//            break;
//        default:{
//            break;
//        }
//
//    }
//    right_gimbal_mode_change();
//}
//
//static void right_gimbal_mode_change()
//{
//    if(rightgimbal.mode == RIGHT_GIMBAL_ACTIVE)
//    {    //自瞄锁定 0x31表示自瞄数据无效
//        if(robot_ctrl.target_lock==0x31 && (detect_list[DETECT_AUTO_AIM].status==ONLINE))
//        {
//            rightgimbal.last_mode=RIGHT_GIMBAL_ACTIVE;
//            rightgimbal.mode=RIGHT_GIMBAL_AUTO;
//        }
//    }
//    else if(rightgimbal.mode == RIGHT_GIMBAL_PATROL)
//    {   //自瞄判定
//        if(robot_ctrl.target_lock==0x31 && (detect_list[DETECT_AUTO_AIM].status==ONLINE))
//        {
//            rightgimbal.last_mode=RIGHT_GIMBAL_PATROL;
//            rightgimbal.mode=RIGHT_GIMBAL_AUTO;
//        }
//    }
//    else if(rightgimbal.mode==RIGHT_GIMBAL_AUTO)
//    {   //自瞄失败判定
//        if((detect_list[DETECT_AUTO_AIM].status==ONLINE)||robot_ctrl.target_lock ==0x32)
//        {
//            rightgimbal.last_mode=RIGHT_GIMBAL_AUTO;
//            rightgimbal.mode=RIGHT_GIMBAL_ACTIVE;
//        }
//    }
//}
//
//
//static void right_gimbal_can_send_back_mapping() {
////    int16_t *real_motor_give_current[4];
////    real_motor_give_current[0] = &launcher.motor_launcher[FIRE_RIGHT].give_current;
////    real_motor_give_current[1] = &rightgimbal.motor_gimbal[PITCH].give_current;
////    real_motor_give_current[2] = &launcher.motor_launcher[FIRE_LEFT].give_current;
////    real_motor_give_current[3] = &launcher.motor_launcher[BARRLE].give_current;
////
////    can_send_dji_motor(CAN_1,
////                       CAN_DJI_MOTOR_0x1FF_ID,
////                       rightgimbal.motor_gimbal[YAW].give_current,
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
//static  void right_gimbal_back_handle()
//{
//    if(rightgimbal.right_yaw_is_back == 0)
//    {
//        rightgimbal.yaw.absolute_angle_set=rightgimbal.motor_gimbal[YAW].relative_angle_get;
//        rightgimbal.motor_gimbal[YAW].relative_angle_set=0;
//        rightgimbal.right_yaw_is_back=1;
//    }
//    //这里的-2是要测的！暂时写-2
//    if(rightgimbal.pitch.absolute_angle_get>=-2 && rightgimbal.pitch.absolute_angle_get<=2)
//    {
//        rightgimbal.right_pitch_is_back=1;
//    }
//    else{
//        rightgimbal.pitch.absolute_angle_set=0;
//    }
//
//}
//
//static void right_gimbal_active_handle()
//{   //注意！这里的set就是pid下面的输入，遥控器的数值。
//    rightgimbal.yaw.absolute_angle_set+=(float32_t)-rc_ctrl.rc.ch[YAW_CHANNEL]*RC_TO_YAW*GIMBAL_RC_MOVE_RATIO_YAW;
//    rightgimbal.pitch.absolute_angle_set+=(float32_t)rc_ctrl.rc.ch[PITCH_CHANNEL]*RC_TO_PITCH*GIMBAL_RC_MOVE_RATIO_PIT;
//
//    rightgimbal.pitch.absolute_angle_set= fp32_constrain(rightgimbal.pitch.absolute_angle_set,MIN_RELA_ANGLE,MAX_RELA_ANGLE);
//    //思考一下小云台的pitch限位要和大云台的pitch一样吗  rela是什么
//    //应该不一样的系数 小云台的pitch不能360旋转的，不然会撞，应该是除了另外一个云台的交界处，其它坐标系都可以
//    //然后yaw的限位也是在这里设置吗
//}
//
//static void right_gimbal_auto_handle()
//{
//    average_add(&MF_auto_pitch,robot_ctrl.pitch);
//    rightgimbal.yaw.absolute_angle_set=robot_ctrl.yaw;
//    rightgimbal.pitch.absolute_angle_set=MF_auto_pitch.aver_num;
//    //这里的abs是绝对值
//    rightgimbal.pitch.absolute_angle_set= fp32_constrain(rightgimbal.pitch.absolute_angle_set,MIN_ABS_ANGLE,MAX_ABS_ANGLE);
//}
//
//static void right_gimbal_relax_handle()
//{
//    rightgimbal.motor_gimbal[YAW].give_current=0;
//    rightgimbal.motor_gimbal[PITCH].give_current=0;
//    launcher_relax_handle();
//}
//
//static void right_gimbal_patrol_handle()
//{
//    if(HAL_GetTick()-nav_time>=1000)
//    {
//        if(robot_ctrl.left_patrol_angle!=robot_ctrl.right_patrol_angle)
//        {
//
//            if(right_gimbal_left_flag)
//            {
//                //为什么角度的获取和云台转动系数有关
//                rightgimbal.yaw.absolute_angle_set+=GIMBAL_YAW_PATROL_SPEED;
//                if(rightgimbal.yaw.absolute_angle_get>=(fp32)robot_ctrl.left_patrol_angle){
//                    right_gimbal_left_flag=0;
//                    right_gimbal_right_flag=1;
//                }
//            }
//    // 这里的robot_ctrl.right_patrol_angle是最大值的意思还是什么
//            else if(right_gimbal_right_flag)
//            {
//                rightgimbal.yaw.absolute_angle_set-=GIMBAL_YAW_PATROL_SPEED;
//                if(rightgimbal.yaw.absolute_angle_get<=(fp32)robot_ctrl.right_patrol_angle)
//                {
//                    right_gimbal_right_flag=0;
//                }
//            }
//            rightgimbal.pitch.absolute_angle_set=-2;
//             //要不要写pitch巡逻
//        }
//        else
//        {   //行为树发送的左右巡逻角度相同，进入360 记得有个角度限制
//            //等出车了 测一下都为0时会不会相撞
//            //一个云台180
//            rightgimbal.yaw.absolute_angle_set+=GIMBAL_YAW_PATROL_SPEED;
//            rightgimbal.pitch.absolute_angle_set=-2;
//            right_gimbal_ctrl_loop_cal(ABSOLUTE_CTRL);
//        }
//    }
//    else
//    {  //导航强制云台巡逻
//        rightgimbal.yaw.absolute_angle_set+=GIMBAL_YAW_PATROL_SPEED;
//        rightgimbal.pitch.absolute_angle_set=-2;
//        right_gimbal_ctrl_loop_cal(AUTO_CTRL);
//    }
//
//}
//
////云台电机闭环控制函数
////mode 0为手动控制yaw
////mode 1为导航控制yaw
//static void right_gimbal_ctrl_loop_cal(int mode)
//{
//
//    if(mode == ABSOLUTE_CTRL)//手动
//    {
//        //右云台绕圈进入绝对角循环 我只想让它0到180
//        //左云台0到-180
////        if(rightgimbal.yaw.absolute_angle_set>=180){
////            rightgimbal.yaw.absolute_angle_set-=180;
////        }
////        else if(rightgimbal.yaw.absolute_angle_set<=-180){
////            rightgimbal.yaw.absolute_angle_set+=360;
////        }
//        first_order_filter_cali(&right_yaw_angle_in,rightgimbal.yaw.absolute_angle_set);
//        first_order_filter_cali(&right_yaw_angle,rightgimbal.motor_gimbal[YAW].relative_angle_get);
//        rightgimbal.motor_gimbal[YAW].ecd_set= pid_loop_calc(&rightgimbal.motor_gimbal[YAW].angle_p,
//                                                             right_yaw_angle.out,
//                                                             right_yaw_angle_in.out,
//                                                             180-critical_value,
//                                                             critical_value);
//        first_order_filter_cali(&right_yaw_speed,rightgimbal.motor_gimbal[YAW].motor_info.speed_rpm);
//        first_order_filter_cali(&right_yaw_speed_in,rightgimbal.motor_gimbal[YAW].ecd_set);
//        rightgimbal.motor_gimbal[YAW].give_current=(int16_t)-pid_calc(&rightgimbal.motor_gimbal[YAW].speed_p,
//                                                                      rightgimbal.motor_gimbal[YAW].motor_info.speed_rpm,
//                                                                      right_yaw_speed_in.out);
//    }
//    else if(mode == AUTO_CTRL)//强制
//    {
////        if(rightgimbal.yaw.absolute_angle_set>=180){
////            rightgimbal.yaw.absolute_angle_set-=360;
////        }
////        else if(rightgimbal.yaw.absolute_angle_set<=-180) {
////            rightgimbal.yaw.absolute_angle_set += 360;
////        }
//
//        first_order_filter_cali(&right_yaw_angle_in,rightgimbal.yaw.absolute_angle_set);
//        first_order_filter_cali(&right_yaw_angle,rightgimbal.motor_gimbal[YAW].relative_angle_get);
//        rightgimbal.motor_gimbal[YAW].ecd_set= pid_loop_calc(&rightgimbal.motor_gimbal[YAW].angle_p,
//                                                             right_yaw_angle.out,
//                                                             right_yaw_angle_in.out,
//                                                             180-critical_value,
//                                                             critical_value);
//        first_order_filter_cali(&right_yaw_speed,rightgimbal.motor_gimbal[YAW].motor_info.speed_rpm);
//        first_order_filter_cali(&right_yaw_speed_in,rightgimbal.motor_gimbal[YAW].ecd_set);
//        rightgimbal.motor_gimbal[YAW].give_current=(int16_t)-pid_calc(&rightgimbal.motor_gimbal[YAW].speed_p,
//                                                                      rightgimbal.motor_gimbal[YAW].motor_info.speed_rpm,
//                                                                      right_yaw_speed_in.out);
//
//    }
//    rightgimbal.motor_gimbal[PITCH].ecd_set= pid_calc(&rightgimbal.motor_gimbal[PITCH].angle_p,
//                                                      rightgimbal.pitch.absolute_angle_get,
//                                                      rightgimbal.pitch.absolute_angle_set);
//    first_order_filter_cali(&right_pitch_speed_in,rightgimbal.motor_gimbal[PITCH].ecd_set);
//    //rightgimbal.motor_gimbal[PITCH].relative_angle_get=((fp32)rightgimbal.motor_gimbal[PITCH].motor_info.speed_rpm*PI)/30.0f;
//    first_order_filter_cali(&right_yaw_first_order_set,rightgimbal.absolute_ecd_pitch);
//
//    rightgimbal.motor_gimbal[PITCH].give_current=(int16_t)-pid_calc(&rightgimbal.motor_gimbal[PITCH].speed_p,
//                                                                    right_pitch_current_first_order_set.out,
//                                                                    right_pitch_speed_in.out);
//    first_order_filter_cali(&right_pitch_first_order_set,rightgimbal.motor_gimbal[PITCH].give_current);
//    rightgimbal.motor_gimbal[PITCH].give_current=(int16_t)right_pitch_first_order_set.out;
//}
////云台角度更新
//static void right_gimbal_angle_update()
//{
//    rightgimbal.pitch.absolute_angle_get=motor_ecd_to_angle_change(rightgimbal.motor_gimbal[PITCH].motor_info.ecd,
//                                                                   rightgimbal.motor_gimbal[PITCH].motor_info.offset_ecd);
//    rightgimbal.motor_gimbal[PITCH].relative_angle_get= motor_ecd_to_angle_change(rightgimbal.motor_gimbal[PITCH].motor_info.ecd,
//                                                                                  rightgimbal.motor_gimbal[PITCH].motor_info.offset_ecd);
//
//    rightgimbal.yaw.absolute_angle_get=-motor_ecd_to_angle_change(rightgimbal.motor_gimbal[YAW].motor_info.ecd,
//                                                                  rightgimbal.motor_gimbal[YAW].motor_info.offset_ecd);
//    rightgimbal.motor_gimbal[YAW].relative_angle_get=-motor_ecd_to_angle_change(rightgimbal.motor_gimbal[YAW].motor_info.ecd,
//                                                                                rightgimbal.motor_gimbal[YAW].motor_info.offset_ecd);
//
//    rightgimbal.absolute_ecd_yaw=(fp32)INS_gyro[2];
//    rightgimbal.absolute_ecd_pitch=(fp32)INS_gyro[0];
//
//    vision_data.yaw=rightgimbal.yaw.absolute_angle_get;
//    vision_data.pitch=rightgimbal.pitch.absolute_angle_get;
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