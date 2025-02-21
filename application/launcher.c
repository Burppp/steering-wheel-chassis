
//发射机构由左拨杆控制
//开启云台之后，左拨杆向上打一次，开启摩擦轮，拨杆回中后再向上打一次关闭摩擦轮
//在摩擦轮开启时，左拨杆向下，开启拨轮，拨杆回中关闭拨轮

#include "launcher.h"

uint8_t rc_last_sw_L;
uint8_t rc_last_sw_R;
uint8_t blocked_flag;
uint8_t barrle_init_to_block_flag;
uint8_t barrle_init_flag;
uint8_t reverse_flag;
uint32_t continue_shoot_time;//遥控器左边拨杆down的持续时间
uint32_t blocked_start_time;
uint32_t barrle_init_block_time;
uint32_t reverse_start_time;
first_order_filter_type_t filter_trigger_rpm_in;
first_order_filter_type_t filter_barrle_rpm_in;
static int fire_time=0;
static int32_t barrle_total_ecd_ref;
extern vision_t vision_data;
extern RC_ctrl_t rc_ctrl;
extern gimbal_t gimbal;
extern robot_ctrl_info_t robot_ctrl;

bool trigger_is_blocked();

void trigger_block_handle();

void launcher_shooting_limit();

void switch_barrel();

void change_barrel(char mode);

//通过赋值进行发射机构的初始化
launcher_t launcher;


void launcher_init() {
    blocked_flag = false;//堵转标志位置0

    reverse_flag = false;//反转标志位置0

    barrle_init_flag = false;

    launcher.trigger_cmd = SHOOT_CLOSE;//初始时发射机构默认关闭

    launcher.fire_last_mode = Fire_OFF;//初始时摩擦轮默认关闭

    launcher.fire_mode = Fire_OFF;//初始时摩擦轮默认关闭

    launcher.barrle_cmd = CHANGE_OK;

    //发射机构电机PID初始化
    launcher.motor_launcher[FIRE_RIGHT].speed_p.p = SHOOT_FIRE_R_PID_KP;
    launcher.motor_launcher[FIRE_RIGHT].speed_p.i = SHOOT_FIRE_R_PID_KI;
    launcher.motor_launcher[FIRE_RIGHT].speed_p.d = SHOOT_FIRE_R_PID_KD;
    launcher.motor_launcher[FIRE_RIGHT].speed_p.max_output = SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.motor_launcher[FIRE_RIGHT].speed_p.integral_limit = SHOOT_FIRE_R_PID_MAX_IOUT;

    launcher.motor_launcher[FIRE_LEFT].speed_p.p = SHOOT_FIRE_L_PID_KP;
    launcher.motor_launcher[FIRE_LEFT].speed_p.i = SHOOT_FIRE_L_PID_KI;
    launcher.motor_launcher[FIRE_LEFT].speed_p.d = SHOOT_FIRE_L_PID_KD;
    launcher.motor_launcher[FIRE_LEFT].speed_p.max_output = SHOOT_FIRE_L_PID_MAX_OUT;
    launcher.motor_launcher[FIRE_LEFT].speed_p.integral_limit = SHOOT_FIRE_L_PID_MAX_IOUT;

    launcher.motor_launcher[TRIGGER].angle_p.p = SHOOT_TRI_ANGLE_PID_KP;
    launcher.motor_launcher[TRIGGER].angle_p.i = SHOOT_TRI_ANGLE_PID_KI;
    launcher.motor_launcher[TRIGGER].angle_p.d = SHOOT_TRI_ANGLE_PID_KD;
    launcher.motor_launcher[TRIGGER].angle_p.max_output = SHOOT_TRI_ANGLE_PID_MAX_OUT;
    launcher.motor_launcher[TRIGGER].angle_p.integral_limit = SHOOT_TRI_ANGLE_PID_MAX_IOUT;

    launcher.motor_launcher[TRIGGER].speed_p.p = SHOOT_TRI_SPEED_PID_KP;
    launcher.motor_launcher[TRIGGER].speed_p.i = SHOOT_TRI_SPEED_PID_KI;
    launcher.motor_launcher[TRIGGER].speed_p.d = SHOOT_TRI_SPEED_PID_KD;
    launcher.motor_launcher[TRIGGER].speed_p.max_output = SHOOT_TRI_SPEED_PID_MAX_OUT;
    launcher.motor_launcher[TRIGGER].speed_p.integral_limit = SHOOT_TRI_SPEED_PID_MAX_IOUT;

    //最开始的编码值作为拨轮电机的校准值
    launcher.motor_launcher[TRIGGER].motor_info.total_ecd = launcher.motor_launcher[TRIGGER].motor_info.offset_ecd = launcher.motor_launcher[TRIGGER].motor_info.ecd;
    first_order_filter_init(&filter_trigger_rpm_in, 1, 1);

    launcher.motor_launcher[BARRLE].angle_p.p = SHOOT_CNG_ANGLE_PID_KP;
    launcher.motor_launcher[BARRLE].angle_p.i = SHOOT_CNG_ANGLE_PID_KI;
    launcher.motor_launcher[BARRLE].angle_p.d = SHOOT_CNG_ANGLE_PID_KD;
    launcher.motor_launcher[BARRLE].angle_p.max_output = SHOOT_CNG_ANGLE_PID_MAX_OUT;
    launcher.motor_launcher[BARRLE].angle_p.integral_limit = SHOOT_CNG_ANGLE_PID_MAX_IOUT;

    launcher.motor_launcher[BARRLE].speed_p.p = SHOOT_CNG_SPEED_PID_KP;
    launcher.motor_launcher[BARRLE].speed_p.i = SHOOT_CNG_SPEED_PID_KI;
    launcher.motor_launcher[BARRLE].speed_p.d = SHOOT_CNG_SPEED_PID_KD;
    launcher.motor_launcher[BARRLE].speed_p.max_output = SHOOT_CNG_SPEED_PID_MAX_OUT;
    launcher.motor_launcher[BARRLE].speed_p.integral_limit = SHOOT_CNG_SPEED_PID_MAX_IOUT;

    //最开始的编码值作为换枪管电机的校准值
    launcher.motor_launcher[BARRLE].motor_info.total_ecd = launcher.motor_launcher[BARRLE].motor_info.offset_ecd = launcher.motor_launcher[BARRLE].motor_info.ecd;
    first_order_filter_init(&filter_barrle_rpm_in, 1, 1);
}

void launcher_mode_set() {

    //摩擦轮关闭时,做拨杆向上拨一下开启摩擦轮
    if (((!switch_is_up(rc_last_sw_L) && switch_is_up(rc_ctrl.rc.s[RC_s_L]))||
        (Referee.GameState.game_progress==9||
         Referee.GameState.game_progress==9||
         Referee.GameState.game_progress==9||
         Referee.GameState.game_progress==9))
         && launcher.fire_mode == Fire_OFF)
    {
        launcher.fire_mode = Fire_ON;
        launcher.fire_last_mode = Fire_OFF;
    }
        //摩擦轮开启时,做拨杆向上拨一下关闭摩擦轮
    else if (!switch_is_up(rc_last_sw_L) && switch_is_up(rc_ctrl.rc.s[RC_s_L]) && launcher.fire_mode == Fire_ON)
    {
        launcher.fire_mode = Fire_OFF;
        launcher.fire_last_mode = Fire_ON;
    }

    //拨轮控制
    if (launcher.fire_mode == Fire_ON && (switch_is_down(rc_ctrl.rc.s[RC_s_L])|| robot_ctrl.fire_command == 0x31 ))
    {                               //兼具调试时手动开火和自瞄有效时自动开火
        if ((robot_ctrl.fire_command == 0x31
         && (Referee.GameState.game_progress == 4 || Referee.GameState.game_progress == 9))
         || (!switch_is_down(rc_last_sw_L) && switch_is_down(rc_ctrl.rc.s[RC_s_L])))
        {
            launcher.trigger_cmd = SHOOT_SINGLE;
            continue_shoot_time = HAL_GetTick();//计时
        }
        //计时达到触发连发的时间
        if ((robot_ctrl.fire_command == 0x31
          && (Referee.GameState.game_progress == 4 || Referee.GameState.game_progress == 9))
          || (switch_is_down(rc_ctrl.rc.s[RC_s_L])) && CONTINUES_SHOOT_TIMING_COMPLETE())
        {
            launcher.trigger_cmd = SHOOT_CONTINUES;
        }
    }
    else if(robot_ctrl.fire_command==0x32)
    {
        fire_time++;
        if(fire_time>=20)
        {
            launcher.trigger_cmd = SHOOT_CLOSE;
            fire_time = 0;
        }
    }
    else {
        launcher.trigger_cmd = SHOOT_CLOSE;
    }
    rc_last_sw_L = rc_ctrl.rc.s[RC_s_L];
    rc_last_sw_R = rc_ctrl.rc.s[RC_s_R];
}

//发射机构控制
void launcher_control() {

    static int32_t total_ecd_ref;
//        switch_barrel();//换枪管

        if (launcher.fire_mode == Fire_ON) {
            launcher.motor_launcher[FIRE_LEFT].rpm_set = FIRE_SPEED_MAX;
            launcher.motor_launcher[FIRE_RIGHT].rpm_set = - FIRE_SPEED_MAX;
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1080);//
            laser_on();
        } else {
            laser_off();
            launcher.motor_launcher[FIRE_LEFT].rpm_set = 0;
            launcher.motor_launcher[FIRE_RIGHT].rpm_set = 0;
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1500);//
        }
        launcher_shooting_limit();

        if (launcher.trigger_cmd == SHOOT_CLOSE) {
            launcher.motor_launcher[TRIGGER].rpm_set = 0;
        } else if (launcher.trigger_cmd == SHOOT_SINGLE) {
            launcher.trigger_cmd = SHOOT_ING;//进入正在单发状态
            total_ecd_ref = launcher.motor_launcher[TRIGGER].motor_info.total_ecd + DEGREE_45_TO_ENCODER;
        } else if (launcher.trigger_cmd == SHOOT_ING) {
            launcher.motor_launcher[TRIGGER].rpm_set = pid_calc(&launcher.motor_launcher[TRIGGER].angle_p,
                                                                (float)launcher.motor_launcher[TRIGGER].motor_info.total_ecd,
                                                                (float)total_ecd_ref);
        } else if (launcher.trigger_cmd == SHOOT_CONTINUES) {
            launcher.motor_launcher[TRIGGER].rpm_set = TRIGGER_CONTINUES_SPEED;
            trigger_block_handle();
        }

        first_order_filter_cali(&filter_trigger_rpm_in, launcher.motor_launcher[TRIGGER].motor_info.speed_rpm);
        launcher.motor_launcher[TRIGGER].give_current = (int16_t)pid_calc(&launcher.motor_launcher[TRIGGER].speed_p,
                                                                          launcher.motor_launcher[TRIGGER].motor_info.speed_rpm,
                                                                          launcher.motor_launcher[TRIGGER].rpm_set);

        first_order_filter_cali(&filter_barrle_rpm_in, launcher.motor_launcher[BARRLE].motor_info.speed_rpm);
        launcher.motor_launcher[BARRLE].give_current = (int16_t)pid_calc(&launcher.motor_launcher[BARRLE].speed_p,
                                                                         launcher.motor_launcher[BARRLE].motor_info.speed_rpm,
                                                                         launcher.motor_launcher[BARRLE].rpm_set);

        launcher.motor_launcher[FIRE_LEFT].give_current = (int16_t)pid_calc(&launcher.motor_launcher[FIRE_LEFT].speed_p,
                                                                            launcher.motor_launcher[FIRE_LEFT].motor_info.speed_rpm,
                                                                            launcher.motor_launcher[FIRE_LEFT].rpm_set);
        launcher.motor_launcher[FIRE_RIGHT].give_current = (int16_t)pid_calc(&launcher.motor_launcher[FIRE_RIGHT].speed_p,
                                                                             launcher.motor_launcher[FIRE_RIGHT].motor_info.speed_rpm,
                                                                             launcher.motor_launcher[FIRE_RIGHT].rpm_set);
}

//发射机构失能
void launcher_relax_handle() {
    launcher.motor_launcher[TRIGGER].give_current = 0;
    launcher.motor_launcher[FIRE_LEFT].give_current = 0;
    launcher.motor_launcher[FIRE_RIGHT].give_current = 0;
}


//判断是否堵转
bool trigger_is_blocked() {
    if (TRIGGER_CONTINUES_SPEED > 0) {
        if (blocked_flag == false && launcher.motor_launcher[TRIGGER].motor_info.speed_rpm <= 0.3 * TRIGGER_CONTINUES_SPEED) {
            blocked_start_time = HAL_GetTick();//获取堵转开始时间
            blocked_flag = true;
        }

        //标识位为1时，已经开始堵转，判断是否堵转达到一定时间，若达到，则判定堵转
        if (blocked_flag == true && launcher.motor_launcher[TRIGGER].motor_info.speed_rpm <= 0.3 * TRIGGER_CONTINUES_SPEED) {
            if (CONTINUES_BLOCKED_JUDGE()) {
                blocked_flag = false;
                return true;
            }
        }
        return false;
    } else if (TRIGGER_CONTINUES_SPEED < 0) {
        //在标识位为0时，电机转速低于阈值时，判定堵转开始
        if (blocked_flag == false && launcher.motor_launcher[TRIGGER].motor_info.speed_rpm >= 0.3 * TRIGGER_CONTINUES_SPEED) {
            blocked_start_time = HAL_GetTick();//获取堵转开始时间
            blocked_flag = true;
        }

        //标识位为1时，已经开始堵转，判断是否堵转达到一定时间，若达到，则判定堵转
        if (blocked_flag == true && launcher.motor_launcher[TRIGGER].motor_info.speed_rpm >= 0.3 * TRIGGER_CONTINUES_SPEED) {
            if (CONTINUES_BLOCKED_JUDGE()) {
                blocked_flag = false;
                return true;
            }
        }
        return false;
    }
}

//堵转处理函数
void trigger_block_handle() {
    //判断堵转并且反转标识为0时
    if (trigger_is_blocked() && reverse_flag == false) {
        reverse_flag = true;//判定开始反转
        reverse_start_time = HAL_GetTick();//获取开始反转时间
    }

    //判定反转开始并且时间没有达到反转结束时间
    if (reverse_flag == true && TRIGGER_REVERSE_TIME_JUDGE()) {
        launcher.motor_launcher[TRIGGER].rpm_set = TRIGGER_REVERSE_SPEED;//拨单电机设置为反转速度
    } else {
        reverse_flag = false;
    }
}

//发射机构热量限制
void launcher_shooting_limit() {
    if ((Referee.BulletRemaining.bullet_remaining_num_17mm <= 10 ||
        (Referee.PowerHeatData.shooter_17mm_1_barrel_heat>=HEAT_MAX && Referee.PowerHeatData.shooter_17mm_2_barrel_heat>=HEAT_MAX))
         && Referee.GameState.game_progress!=0)
    {
        launcher.trigger_cmd = SHOOT_CLOSE;
    }
}

void barrle_init_pose() {
    launcher.motor_launcher[BARRLE].rpm_set = 1000;
        //电机转速低于阈值时，判定
        if (barrle_init_to_block_flag==false ) {
            barrle_init_block_time = HAL_GetTick();//获取堵转开始时间
            barrle_init_to_block_flag = true;
        }

        //标识位为1时，已经开始堵转，判断是否堵转达到一定时间，若达到，则判定堵转
        if (barrle_init_to_block_flag == true && launcher.motor_launcher[BARRLE].motor_info.speed_rpm == 0) {
            if (BARRLE_INIT_JUDGE()) {
                barrle_init_block_time = false;
                barrle_total_ecd_ref = launcher.motor_launcher[BARRLE].motor_info.total_ecd;
                launcher.motor_launcher[BARRLE].rpm_set = 0;
                barrle_init_flag = true;
            }
        }
}

//换枪管控制实现部分
void change_barrel(char mode)
{

    if(mode == TURN_LEFT)
    {
        if(launcher.barrle_cmd == CHANGE_SINGLE)
        {
            barrle_total_ecd_ref = launcher.motor_launcher[BARRLE].motor_info.total_ecd + DEGREE_BARRLE_TO_ENCODER;
//            buzzer_on(1,40000);
            launcher.barrle_cmd = CHANGE_ING;
        }
    }
    else if(mode == TURN_RIGHT)
    {
        if(launcher.barrle_cmd == CHANGE_SINGLE)
        {
            barrle_total_ecd_ref = launcher.motor_launcher[BARRLE].motor_info.total_ecd - DEGREE_BARRLE_TO_ENCODER;
//            buzzer_on(1,40000);
            launcher.barrle_cmd = CHANGE_ING;
        }
    }
    if((float)launcher.motor_launcher[BARRLE].motor_info.total_ecd - (float)barrle_total_ecd_ref < 1000
       && (float)launcher.motor_launcher[BARRLE].motor_info.total_ecd - (float)barrle_total_ecd_ref > -1000)
    {
        launcher.barrle_cmd = CHANGE_OK;
    }
}


//换枪管
void switch_barrel() { //装舵机的时候记得->叫机械->来叫电控-> 做“舵机调零”
    static int count = 0;
    static int L_flag = 0;//默认启动时为左枪管
    static int R_flag = 1;
    static int last_fire_mode;
    static int last_trigger_cmd;

    static int stage_0_flag = 1;//枪管状态，0为轮换中，1为就绪
    static int stage_1_flag = 0;//等待摩擦轮停止，0为已停止，1为正在停止
    static int stage_2_flag = 0;//等待枪管轮换，0为已完成，1为未完成
    static int stage_3_flag = 0;//恢复状态，0为NO，1为YES

    //判断
//    if (Referee.PowerHeatData.shooter_heat0 >= 280 && switch_flag == 0)
//    {//左枪管热量耗尽
//        switch_flag = 1;
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, RIGHT_BARREL_PWM);
//    } else if (Referee.PowerHeatData.shooter_heat1 >= 280 && switch_flag == 0)
//    {//右枪管热量耗尽
//        switch_flag = 1;
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, LEFT_BARREL_PWM);
//    }

    if(barrle_init_flag == false)
    {
        barrle_init_pose();   //在机械限位还没有之前，在调试时暂时关闭
    }
    else {
        /*判断是否轮换*/
        if (stage_0_flag) {
            //线上模式
            if (Referee.PowerHeatData.shooter_17mm_1_barrel_heat >= HEAT_MAX || Referee.PowerHeatData.shooter_17mm_2_barrel_heat >= HEAT_MAX) {
                //进入下一阶段
                stage_0_flag = 0;
                stage_1_flag = 1;
                //保存轮换前发射状态
                last_fire_mode = launcher.fire_mode;
                last_trigger_cmd = launcher.trigger_cmd;
                launcher.barrle_cmd = CHANGE_SINGLE;
            }

                //线下模式
                //手动模式下（右开关为中），遥控器小拨轮往上打为手动换枪管
            else if (rc_ctrl.rc.ch[4] <= -600 && switch_is_mid(rc_ctrl.rc.s[RC_s_R])) {
                //进入下一阶段
                stage_0_flag = 0;
                stage_1_flag = 1;
                //保存轮换前发射状态
                last_fire_mode = launcher.fire_mode;
                last_trigger_cmd = launcher.trigger_cmd;
                launcher.barrle_cmd = CHANGE_SINGLE;

            }
            else {
                //不需要轮换则保持当前状态
                if (L_flag)
                    change_barrel(CHANGE_OK);
                else if (R_flag)
                    change_barrel(CHANGE_OK);
            }
        }

        /*停止摩擦轮*/
        if (stage_1_flag) {
            //停止发射
            launcher.fire_mode = Fire_OFF;
            launcher.trigger_cmd = SHOOT_CLOSE;

            count++;
            if (count >= STAGE_1_WAITING_TIME) {
                //进入下一阶段
                stage_1_flag = 0;
                stage_2_flag = 1;
                count = 0;
                if (L_flag) {
                    change_barrel(TURN_RIGHT);
                    L_flag = 0;
                    R_flag = 1;
                } else if (R_flag) {
                    change_barrel(TURN_LEFT);
                    L_flag = 1;
                    R_flag = 0;
                }
            }
        }

        /*等待枪管轮换*/
        if (stage_2_flag) {
            //停止发射
            launcher.fire_mode = Fire_OFF;
            launcher.trigger_cmd = SHOOT_CLOSE;
            count++;
            if (count >= STAGE_2_WAITING_TIME || launcher.barrle_cmd == CHANGE_OK) {
                //进入下一阶段
                stage_2_flag = 0;
                stage_3_flag = 1;
                count = 0;
            }
            if ((float) launcher.motor_launcher[BARRLE].motor_info.total_ecd - (float) barrle_total_ecd_ref < 3000
                && (float) launcher.motor_launcher[BARRLE].motor_info.total_ecd - (float) barrle_total_ecd_ref > -3000) {
                buzzer_off();
                launcher.barrle_cmd=CHANGE_OK;
            }
        }

        //恢复状态
        if (stage_3_flag) {
            //进入第一阶段
            stage_3_flag = 0;
            stage_0_flag = 1;
            //恢复状态
            //也可改为换完枪管后为不发射状态
            launcher.fire_mode = last_fire_mode;
            launcher.trigger_cmd = last_trigger_cmd;
        }
        launcher.motor_launcher[BARRLE].rpm_set = pid_calc(&launcher.motor_launcher[BARRLE].angle_p,
                                                           (float) launcher.motor_launcher[BARRLE].motor_info.total_ecd,
                                                           (float) barrle_total_ecd_ref);
    }
}