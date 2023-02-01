#ifndef CHASSISTASK_H
#define CHASSISTASK_H

#include "Remote_Control.h"
#include "pid.h"
#include "CAN_Receive.h"
#include "user_lib.h"
#include "stm32.h"


//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

#define CHASSIS_RC_DEADLINE 10

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.2f
#define MOTOR_DISTANCE_TO_CENTER   0.32 //0.32f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 1
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002

//底盘低通滤波系数
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//X，Y角度与遥控器输入比例
#define X_RC_SEN   0.0006f
#define Y_RC_SEN -0.0005f //0.005

//Y,Y角度和鼠标输入的比例
#define X_Mouse_Sen 0.0002f
#define Y_Mouse_Sen 0.00025f

//X,Y控制通道
#define X_Channel 2
#define Y_Channel 3

//电机码盘值最大以及中值
#define Half_ecd_range 395  //395  7796
#define ecd_range 8191

//编码-弧度
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.00076708402f //      2*  PI  /8192
#endif

//底盘电机最大速度
#define MAX_WHEEL_SPEED 4.5f

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//底盘电机速度环PID
#define CHASSIS_KP 6.f  //78.6432--62.91456
#define CHASSIS_KI 0.5f //118098f
#define CHASSIS_KD 30.f //
#define CHASSIS_MAX_OUT 10000.f
#define CHASSIS_MAX_IOUT 3000.f

//舵电机PID
#define RUDDER_P_P 40.f 
#define RUDDER_P_I 0.f 
#define RUDDER_P_D 0.f
#define RUDDER_P_N 0.f
#define RUDDER_S_P 3.f
#define RUDDER_S_I 0.f
#define RUDDER_S_D 0.f
#define RUDDER_S_N 0.f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP  15     //8.f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 1000.f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  5000  //8.5
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_KF 0.f
#define CHASSIS_FOLLOW_GIMBAL_F_divider 0.0
#define CHASSIS_FOLLOW_GIMBAL_F_out_limit 00.f

//底盘电机功率环PID
#define M3505_MOTOR_POWER_PID_KP 2000.f
#define M3505_MOTOR_POWER_PID_KI 85.f
#define M3505_MOTOR_POWER_PID_KD 0.f
#define M3505_MOTOR_POWER_PID_MAX_OUT 60000.0f
#define M3505_MOTOR_POWER_PID_MAX_IOUT 900.0f

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f


//m3508电机的减速比
#define M3508_MOTOR_REDUCATION 19.2032f

//m3508 rpm change to chassis speed
//m3508转子转速(rpm)转化成底盘速度(m/s)的比例，c=pi*r/(30*k)，k为电机减速比
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR


//m3508转子转速(rpm)转换为输出轴角速度(rad/s)的比例
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.0054533f


//m3508转矩电流(-16384~16384)转为成电机输出转矩(N.m)的比例
//c=20/16384*0.3，0.3为转矩常数(N.m/A)
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f


//单个底盘电机最大力矩
#define MAX_WHEEL_TORQUE 5.5f



//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //底盘跟随云台
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //底盘自主
	CHASSIS_VECTOR_SPIN,                //小陀螺
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //底盘不跟随
  CHASSIS_VECTOR_RAW,									//底盘原始控制

} chassis_mode_e;

typedef struct
{
  fp32 relative_angle;
} Gimbal_data;
typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;
	
    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;
	
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	 
    fp32 out;
		
} chassis_PID_t;
typedef struct
{
    const motor_measure_t *gimbal_motor_measure;   //云台数据
	  Rudder_control rudder_control;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad
	  
	  fp32 control;
    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;    //角速度设定
    fp32 motor_speed;     
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
		fp32  ecd_add;       
		fp32 ecd_set;  
    fp32 ecd_set_final;	
		fp32 last_ecd_set;
		fp32 ecd_error;
		fp32 ecd_error_true;
		fp32 ecd_turn;
    fp32 ecd_change_MIN;
		fp32 wheel_speed;
		fp32 rudder_angle;
		fp32 last_rudder_angle;
		fp32 ecd_zero_set;
		fp32 Judge_Speed_Direction;
} Rudder_Motor_t;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	
	fp32 omg;     //输出轴角速度
	fp32 torque;  //电机扭矩
	PidTypeDef chassis_pid;
} Chassis_Motor_t;

typedef struct
{        
	  const RC_ctrl_t *chassis_rc_ctrl;
	  Chassis_Motor_t motor_chassis[4]; 
	  chassis_mode_e chassis_motor_mode;
    chassis_mode_e last_chassis_motor_mode;
	 
	  PidTypeDef chassis_angle_pid;   //底盘跟随角度pid
  	const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
	  
	  Gimbal_data gimbal_data;
    Rudder_Motor_t Forward_L;
    Rudder_Motor_t Forward_R;
	  Rudder_Motor_t Back_R;
	  Rudder_Motor_t Back_L; 
	  
	  fp32 vx;    //底盘速度 前进方向 前为正，单位 m/s
    fp32 vy;   //底盘速度 左右方向 左为正  单位 m/s
    fp32 wz;  //底盘旋转角速度，逆时针为正 单位 rad/s
	
	  fp32 vx_set;      
    fp32 vy_set;                         
    fp32 wz_set;                         
		fp32 chassis_relative_angle_set; //设置相对云台控制角度
	
	  first_order_filter_type_t chassis_cmd_slow_set_vx;   // 滤波数据
		first_order_filter_type_t chassis_cmd_slow_set_vy;
    
}chassis_move_t;


extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
