////
//// Created by Jackson on 2023/12/3.
////
//
///*include*/
#include "Chassis.h"
//#include "bsp_can.h"
//#include "can_hardwares.h"
//#include "ramp.h"
///*define*/
///*轮子控制映射：                                 解算坐标：      x(前)
//            ****      前       ****                                 |
//           * 2 LF *          * 1 RF *                              |
//            ****              ****                                 |
//                                                                   |
//           左                   右                 y  --------------z-----------
//                                                                   |
//            ****              ****                                 |
//          * 3 LB *          * 4 RB *                               |
//            ****      后      ****                                 |
//
//*/
///*变量*/
//extern RC_ctrl_t rc_ctrl;
//extern QueueHandle_t CHASSIS_motor_queue;
//ramp_function_source_t chassis_auto_vx_ramp;
//ramp_function_source_t chassis_auto_vy_ramp;
//ramp_function_source_t chassis_auto_vw_ramp;
//ramp_function_source_t chassis_3508_ramp[4];
//chassis_t chassis;
//extern gimbal_t gimbal;
////上位机下发数据
//extern robot_ctrl_info_t robot_ctrl;
////底盘解算发送数据
//extern chassis_odom_info_t chassis_odom;
//extern bool our_outpost_destroyed;
//extern uint32_t nav_time;
//extern fp32 INS_angle[3];
////发送机器人id
vision_t vision_data;
//static steering_velocity_vector_t add(steering_velocity_vector_t *a, steering_velocity_vector_t *b);
//static fp32 rotate_ratio_f = ((Wheel_axlespacing + Wheel_spacing) / 2.0f - GIMBAL_OFFSET); //rad
//static fp32 rotate_ratio_b = ((Wheel_axlespacing + Wheel_spacing) / 2.0f + GIMBAL_OFFSET);
//static fp32 wheel_rpm_ratio = 60.0f / (PERIMETER * M3508_DECELE_RATIO); //che lun zhuan su bi
//int32_t outward_area_01[2][2];
//int32_t outward_area_23[2][4];
//
///*      函数及声明   */
//static void chassis_init();
//
//static void chassis_set_mode();
//
//static void chassis_ctrl_info_get();
//
//static void chassis_relax_handle();
//
//static void chassis_auto_handle();
//
//static void chassis_wheel_cal();
//
//static void chassis_wheel_loop_cal();
//
//void chassis_device_offline_handle();
//
//void send_robot_id();
//
//void chassis_follow_gimbal_handle();
//
//void chassis_spin_handle();
//
//static void chassis_power_limit();
//
//void chassis_can_send_back_mapping();
//
//float float_constrain(float Value, float minValue, float maxValue) {
//    if (Value < minValue)
//        return minValue;
//    else if (Value > maxValue)
//        return maxValue;
//    else
//        return Value;
//}
//
///*程序主体*/
//_Noreturn void chassis_task(void const *pvParameters) {
//
//    vTaskDelay(CHASSIS_TASK_INIT_TIME);
//    chassis_init();//底盘初始化
//    send_robot_id(); //发送机器人id
//    //主任务循环
//    while (1) {
//        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误
//        chassis_set_mode();
////        chassis_device_offline_handle();
//        //判断底盘模式选择 决定是否覆盖底盘转速vw;
//        switch (chassis.mode) {
//            case CHASSIS_DEBUG:
//                chassis_ctrl_info_get(); //遥控器获取底盘方向矢量
////                chassis_follow_gimbal_handle();
//                break;
//
//            case CHASSIS_AUTO://自动巡逻
//                chassis_auto_handle();
//                if((HAL_GetTick() - nav_time>=1000)&&Referee.GameState.game_progress==0)
//                    chassis_ctrl_info_get(); //遥控器获取底盘方向矢量 debug!!!!
//                chassis_spin_handle();
//                break;
//
//            case CHASSIS_RELAX:
//                chassis_relax_handle();
//                break;
//        }
//
//        if (chassis.mode != CHASSIS_RELAX) {
//            //底盘解算
//            chassis_wheel_cal();
//            //驱电机闭环
//            chassis_wheel_loop_cal();
//            //功率限制
//            chassis_power_limit();
//            //电机映射
//            chassis_can_send_back_mapping();
//        }
//        else{
//            chassis_can_send_back_mapping();
//        }
//        xTaskResumeAll();
//
//        vTaskDelay(1);
//    }
//
//}
//void send_robot_id()
//{
//    if(Referee.GameRobotStat.robot_id<10)//红色方的ID小于10
//    {
//        vision_data.id=1;
//    }
//    else{
//        vision_data.id=0;
//    }
//}
//void motor_init(chassis_motor_t *chassis_motor) {
//    for (int i = 0; i < 4; i++) {
//        pid_init(&chassis_motor[i].speed_p,
//                 STEERING_WHEEL_CHASSIS_OUT,
//                 STEERING_WHEEL_CHASSIS_IOUT,
//                 STEERING_WHEEL_CHASSIS_KP,
//                 STEERING_WHEEL_CHASSIS_KI,
//                 STEERING_WHEEL_CHASSIS_KD);
//    }
//    pid_init(&chassis.chassis_vw_pid,
//             CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,
//             CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT,
//             CHASSIS_FOLLOW_GIMBAL_PID_KP,
//             CHASSIS_FOLLOW_GIMBAL_PID_KI,
//             CHASSIS_FOLLOW_GIMBAL_PID_KD);
//}
//
//void steering_motor_init(steering_motor_t *steering_motor){
//    for(int i=0;i<4;i++){
//        pid_init(&steering_motor[i].speed_p,
//                 STEERING_WHEEL_STEER_SPEED_OUT,
//                 STEERING_WHEEL_STEER_SPEED_IOUT,
//                 STEERING_WHEEL_STEER_SPEED_KP,
//                 STEERING_WHEEL_STEER_SPEED_KI,
//                 STEERING_WHEEL_STEER_SPEED_KD);
//        pid_init(&steering_motor[i].angle_p,
//                 STEERING_WHEEL_STEER_ANGLE_OUT,
//                 STEERING_WHEEL_STEER_ANGLE_IOUT,
//                 STEERING_WHEEL_STEER_ANGLE_KP,
//                 STEERING_WHEEL_STEER_ANGLE_KI,
//                 STEERING_WHEEL_STEER_ANGLE_KD);
//    }
//
//    pid_init(&chassis.rotate_pid,
//             STEERING_WHEEL_ROTATE_OUT,
//             STEERING_WHEEL_ROTATE_IOUT,
//             STEERING_WHEEL_ROTATE_KP,
//             STEERING_WHEEL_ROTATE_KI,
//             STEERING_WHEEL_ROTATE_KD);
//}
//static void chassis_init(void) {
//    //底盘驱动电机速度环初始化和电机数据结构体获取
//    motor_init(chassis.motor_chassis);
//    steering_motor_init(chassis.motor_steering);
//
//    //初始时底盘模式为失能
//    chassis.mode = chassis.last_mode = CHASSIS_RELAX;
//}
//
//static void chassis_set_mode() {
//
//    if (switch_is_down(rc_ctrl.rc.s[RC_s_R])) {
//        chassis.last_mode = chassis.mode;
//        chassis.mode = CHASSIS_RELAX;
//    }
//    else if (switch_is_mid(rc_ctrl.rc.s[RC_s_R])) {
//        chassis.last_mode = chassis.mode;
//        chassis.mode = CHASSIS_DEBUG;
//    }
//    else if (switch_is_up(rc_ctrl.rc.s[RC_s_R])) {
//        chassis.last_mode = chassis.mode;
//        chassis.mode = CHASSIS_AUTO;
//    }
//    //防止导航发疯后切换为手动模式再切回自动模式时没有对最后发布的控制指令清零导致继续发疯的问题
//    if(chassis.mode == CHASSIS_DEBUG) {
//        robot_ctrl.vx = 0;
//        robot_ctrl.vy = 0;
//        robot_ctrl.vw = 0;
//
//    }
//}
//
//static void chassis_ctrl_info_get() {
//
//    chassis.vx = (float) -rc_ctrl.rc.ch[1] * STEERING_WHEEL_DR16_RC_TO_CHASSIS_VX;
//    chassis.vy = (float) rc_ctrl.rc.ch[0] * STEERING_WHEEL_DR16_RC_TO_CHASSIS_VY;
//
//    chassis.vw = (float) -rc_ctrl.rc.ch[2] * STEERING_WHEEL_DR16_RC_TO_CHASSIS_VW;
//    chassis.chassis_yaw = INS_angle[0];
//
//    chassis.vx = float_constrain(chassis.vx, -STEERING_WHEEL_MAX_VX,
//                                 STEERING_WHEEL_MAX_VX);
//    chassis.vy = float_constrain(chassis.vy, -STEERING_WHEEL_MAX_VY,
//                                 STEERING_WHEEL_MAX_VY);
//    chassis.vw = float_constrain(chassis.vw, -STEERING_WHEEL_MAX_VW,
//                                 STEERING_WHEEL_MAX_VW);
//}
//
//static void chassis_auto_handle() {
//    ramp_calc(&chassis_auto_vx_ramp, robot_ctrl.vx);
//    robot_ctrl.vx = chassis_auto_vx_ramp.out;
//    ramp_calc(&chassis_auto_vy_ramp, robot_ctrl.vy);
//    robot_ctrl.vy = chassis_auto_vy_ramp.out;
//    ramp_calc(&chassis_auto_vw_ramp, robot_ctrl.vw);
//    robot_ctrl.vw = chassis_auto_vw_ramp.out;
//    chassis.vx = robot_ctrl.vx;
//    chassis.vy = robot_ctrl.vy;
//    chassis.vw = robot_ctrl.vw;
//}
//
//
//
////将期望速度转为转子期望转速
//static void chassis_wheel_cal(){
//
//}
//
//void chassis_feedback_update()
//{
//
//    //update odom vel data
////    chassis_odom.vx = ((float)(chassis.motor_chassis[1].motor_info.speed_rpm
////                               + chassis.motor_chassis[2].motor_info.speed_rpm
////                               - chassis.motor_chassis[0].motor_info.speed_rpm
////                               - chassis.motor_chassis[3].motor_info.speed_rpm
////    )*(WHEEL_MOTO_RATE))/4;
////    chassis_odom.vy = ((float)(chassis.motor_chassis[2].motor_info.speed_rpm
////                               - chassis.motor_chassis[1].motor_info.speed_rpm
////                               - chassis.motor_chassis[0].motor_info.speed_rpm
////                               + chassis.motor_chassis[3].motor_info.speed_rpm
////    )*(WHEEL_MOTO_RATE))/4;
////    chassis_odom.vw = ((float )(-chassis.motor_chassis[2].motor_info.speed_rpm
////                                - chassis.motor_chassis[1].motor_info.speed_rpm
////                                - chassis.motor_chassis[0].motor_info.speed_rpm
////                                - chassis.motor_chassis[3].motor_info.speed_rpm
////    )*(WHEEL_MOTO_RATE))/(2*Wheel_axlespacing+2*Wheel_spacing);
////    chassis_odom.relative_yaw = gimbal.motor_gimbal[YAW].relative_angle_get;
////    chassis_odom.gyro_yaw = gimbal.absolute_gyro_yaw;
////    rm_queue_data( CHASSIS_ODOM_CMD_ID,&chassis_odom,sizeof (chassis_odom_info_t));
//}
//
//static void chassis_wheel_loop_cal() {
//
//    chassis.motor_chassis[0].give_current= (int16_t) pid_calc(&chassis.motor_chassis[0].speed_p,
//                                                              chassis.motor_chassis[0].motor_info.speed_rpm,
//                                                              chassis.motor_chassis[0].set_rpm);
//
//
//    chassis.motor_chassis[1].give_current= (int16_t)pid_calc(&chassis.motor_chassis[1].speed_p,
//                                                             chassis.motor_chassis[1].motor_info.speed_rpm,
//                                                             chassis.motor_chassis[1].set_rpm);
//
//
//    chassis.motor_chassis[2].give_current= (int16_t)pid_calc(&chassis.motor_chassis[2].speed_p,
//                                                             chassis.motor_chassis[2].motor_info.speed_rpm,
//                                                             chassis.motor_chassis[2].set_rpm);
//
//    chassis.motor_chassis[3].give_current= (int16_t) pid_calc(&chassis.motor_chassis[3].speed_p,
//                                                              chassis.motor_chassis[3].motor_info.speed_rpm,
//                                                              chassis.motor_chassis[3].set_rpm);
//
//    chassis.motor_steering[0].set_speed=(float) pid_loop_calc(
//            &chassis.motor_steering[0].angle_p,
//            chassis.motor_steering[0].get_angle,
//            chassis.motor_steering[0].set_angle,
//            180,
//            -180
//    );
//    chassis.motor_steering[1].set_speed=(float) pid_loop_calc(
//            &chassis.motor_steering[1].angle_p,
//            chassis.motor_steering[1].get_angle,
//            chassis.motor_steering[1].set_angle,
//            180,
//            -180
//    );
//    chassis.motor_steering[2].set_speed=(float) pid_loop_calc(
//            &chassis.motor_steering[2].angle_p,
//            chassis.motor_steering[2].get_angle,
//            chassis.motor_steering[2].set_angle,
//            180,
//            -180
//    );
//    chassis.motor_steering[3].set_speed=(float) pid_loop_calc(
//            &chassis.motor_steering[3].angle_p,
//            chassis.motor_steering[3].get_angle,
//            chassis.motor_steering[3].set_angle,
//            180,
//            -180
//    );
//
//    chassis.motor_steering[0].give_current =(int16_t) pid_calc(
//            &chassis.motor_steering[0].speed_p,
//            chassis.motor_steering[0].motor_info.speed_rpm,
//            chassis.motor_steering[0].set_speed
//    );
//    chassis.motor_steering[1].give_current =(int16_t) pid_calc(
//            &chassis.motor_steering[1].speed_p,
//            chassis.motor_steering[1].motor_info.speed_rpm,
//            chassis.motor_steering[1].set_speed
//    );
//    chassis.motor_steering[2].give_current =(int16_t) pid_calc(
//            &chassis.motor_steering[2].speed_p,
//            chassis.motor_steering[2].motor_info.speed_rpm,
//            chassis.motor_steering[2].set_speed
//    );
//    chassis.motor_steering[3].give_current =(int16_t) pid_calc(
//            &chassis.motor_steering[3].speed_p,
//            chassis.motor_steering[3].motor_info.speed_rpm,
//            chassis.motor_steering[3].set_speed
//    );
//}
//
////功率控制
//void chassis_power_limit() {
//    int16_t give_current_limit;
//    chassis.chassis_power_limit.total_current = 0;
//    chassis.chassis_power_limit.total_current_limit = 0;
//    fp32 power_buffer = chassis.chassis_power_limit.power_buff;
//    fp32 limit_k;
//    if (detect_list[DETECT_REFEREE].status != ONLINE) {
//        chassis.chassis_power_limit.total_current_limit = CHASSIS_CURRENT_LIMIT_40W;
//    } else {
//        chassis.chassis_power_limit.power_buff = Referee.PowerHeatData.chassis_power_buffer > CHASSIS_POWER_BUFF ?
//                                                 CHASSIS_POWER_BUFF : Referee.PowerHeatData.chassis_power_buffer;
//        chassis.chassis_power_limit.limit_k = chassis.chassis_power_limit.power_buff / CHASSIS_POWER_BUFF;
//
//        if (chassis.chassis_power_limit.power_buff < 100) {
//            chassis.chassis_power_limit.limit_k =
//                    chassis.chassis_power_limit.limit_k * chassis.chassis_power_limit.limit_k;
//        } else {
//            chassis.chassis_power_limit.limit_k =
//                    chassis.chassis_power_limit.limit_k;
//        }
//
//        chassis.chassis_power_limit.total_current_limit =
//                chassis.chassis_power_limit.limit_k * CHASSIS_CURRENT_LIMIT_TOTAL;
//    }
//    for (int i = 0; i < 4; i++) {
//        chassis.chassis_power_limit.total_current += abs(chassis.motor_chassis[i].give_current);
//
//        give_current_limit = chassis.motor_chassis[i].give_current;
//
//        if (chassis.chassis_power_limit.total_current > chassis.chassis_power_limit.total_current_limit) {
//            give_current_limit = (int16_t) ((fp32)chassis.motor_chassis[i].give_current *
//                                            chassis.chassis_power_limit.total_current_limit
//                                            / chassis.chassis_power_limit.total_current);
//        }
//        //计算电流
//        chassis.motor_chassis[i].give_current = give_current_limit;
//    }
//}
////把can接收时对真实电机的映射，在发送控制时映射回去为真实的电机，因为控制函数要按电机ID 1～4发送
//void chassis_can_send_back_mapping(){
//    int16_t *real_steer_motor_give_current[4];
//    int16_t *real_chassis_motor_give_current[4];
//    real_steer_motor_give_current[0] = &chassis.motor_steering[0].give_current;
//    real_steer_motor_give_current[1] = &chassis.motor_steering[1].give_current;
//    real_steer_motor_give_current[2] = &chassis.motor_steering[2].give_current;
//    real_steer_motor_give_current[3] = &chassis.motor_steering[3].give_current;
//    real_chassis_motor_give_current[0] = &chassis.motor_chassis[0].give_current;
//    real_chassis_motor_give_current[1] = &chassis.motor_chassis[1].give_current;
//    real_chassis_motor_give_current[2] = &chassis.motor_chassis[2].give_current;
//    real_chassis_motor_give_current[3] = &chassis.motor_chassis[3].give_current;
//    can_send_dji_motor(
//            CAN_1,
//            CAN_DJI_MOTOR_0x1FF_ID,
//            *real_steer_motor_give_current[0],
//            0,
//            0,
//            *real_steer_motor_give_current[3]
//    );
//    can_send_dji_motor(
//            CAN_2,
//            CAN_DJI_MOTOR_0x1FF_ID,
//            0,
//            *real_steer_motor_give_current[1],
//            *real_steer_motor_give_current[2],
//            0);
//    can_send_dji_motor(
//            CAN_1,
//            CAN_DJI_MOTOR_0x200_ID,
//            0,
//            *real_chassis_motor_give_current[1],
//            *real_chassis_motor_give_current[2],
//            0);
//    can_send_dji_motor(
//            CAN_2,
//            CAN_DJI_MOTOR_0x200_ID,
//            *real_chassis_motor_give_current[0],
//            0,
//            0,
//            *real_chassis_motor_give_current[3]
//    );
//}
//void chassis_device_offline_handle() {
//    if(detect_list[DETECT_REMOTE].status==OFFLINE)
//        chassis.mode=CHASSIS_RELAX;//防止出现底盘疯转
//}
//static void chassis_relax_handle() {
//    for (int i = 0; i < 4; ++i) {
//        chassis.motor_chassis[i].give_current = 0;
//        chassis.motor_steering[i].give_current=0;
//    }
//}
//void chassis_follow_gimbal_handle(){
//    // 以上电时的yaw为中心，yaw不动使底盘去归中到对应的编码器中点，在运动时底盘运动方向为底盘正方向，
//    // 当yaw转动后产生了编码器与编码器中值的偏移使底盘进行“跟随yaw”转动，
//    // 然后将底盘正方向移至yaw所指向的方向，就做到了底盘跟云台然后yaw指哪走哪
//    fp32 yaw_relative_radian=gimbal.motor_gimbal[YAW].relative_angle_get*ANGLE_TO_RAD;//相对角度的弧度
//    fp32 sin_yaw,cos_yaw;
//
//    sin_yaw=(fp32)sin(yaw_relative_radian);
//    cos_yaw=(fp32) cos(yaw_relative_radian);
//
//    fp32 vx_temp=chassis.vx;
//    fp32 vy_temp=chassis.vy;
//
//    //速度矢量分解
//    chassis.vx=cos_yaw*vx_temp-sin_yaw*vy_temp;
//    chassis.vy=(sin_yaw*vx_temp+cos_yaw*vy_temp);
//    chassis.vw = pid_calc(&chassis.chassis_vw_pid, -gimbal.motor_gimbal[YAW].relative_angle_get, 0);
//    VAL_LIMIT(chassis.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);
//}
//
//void chassis_spin_handle()
//{
//    //小陀螺实现分为三步：
//    //1、获取底盘与云台的相对角度θ。底盘绝对角度由YAW轴电机提供，云台绝对角度由云台上的角度传感器提供。
//    //2、根据θ，把整车运动速度（大小和方向）分解到底盘坐标
//    //3、根据底盘坐标速度进行麦轮速度分解，整车效果则表现为按云台坐标运动
////    fp32 yaw_relative_radian=gimbal.motor_gimbal[YAW].relative_angle_get*ANGLE_TO_RAD;//相对角度的弧度
////    fp32 sin_yaw,cos_yaw;
////
////    sin_yaw=(fp32) sin(yaw_relative_radian);
////    cos_yaw=(fp32) cos(yaw_relative_radian);
////
////    fp32 vx_temp=chassis.vx;
////    fp32 vy_temp=chassis.vy;
////    //速度矢量分解
////    chassis.vx=vx_temp*cos_yaw - vy_temp*sin_yaw;
////    chassis.vy=(vx_temp*sin_yaw + vy_temp*cos_yaw);//y is left
////    if((robot_ctrl.spin_command==1&&Referee.GameState.game_progress==1)||(HAL_GetTick() - nav_time <1000)||our_outpost_destroyed == true)
////        chassis.vw = 4.0f;
////    else chassis.vw = 0.0f;
////    VAL_LIMIT(chassis.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);
//
//}