#include "bflb_mtimer.h"
#include "bflb_pwm_v2.h"
#include "bflb_clock.h"
#include "bflb_gpio.h"
#include "board.h"

struct bflb_device_s *pwm;
struct bflb_device_s *gpio;

typedef enum
{
    MODE_LINE_TRACKING,  // 循迹模式
    MODE_OBSTACLE_AVOID, // 避障模式
    MODE_STOPPED         // 停止模式
}RobotMode;
RobotMode current_mode = MODE_LINE_TRACKING;
uint32_t avoidance_start_time = 0;
const uint32_t AVOIDANCE_DURATION = 1000; // 避障动作持续时间(ms)


void Irtracking_Init()
{
    bflb_gpio_init(gpio, GPIO_PIN_0, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);//巡线左
    bflb_gpio_init(gpio, GPIO_PIN_1, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);//巡线右
    bflb_gpio_init(gpio, GPIO_PIN_2, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);//红外避障左
    bflb_gpio_init(gpio, GPIO_PIN_3, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);//红外避障右
    
    bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH1, 0, 0); /* duty = (500-100)/1000 = 40% */
    bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH0, 0, 400); /* duty = (400-200)/1000 = 20% */
    bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH3, 0, 400); /* duty = (999-100)/1000 = 89.9% */
    bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH2, 0, 0); /* duty = (500-0)/1000 = 50% */
    bflb_pwm_v2_channel_positive_start(pwm, PWM_CH1);
    bflb_pwm_v2_channel_positive_start(pwm, PWM_CH0);
    bflb_pwm_v2_channel_positive_start(pwm, PWM_CH3);
    bflb_pwm_v2_channel_positive_start(pwm, PWM_CH2);
    bflb_pwm_v2_start(pwm);
}
uint8_t Left_Irtracking_Get(void)
{
	printf("[巡线]GPIO_PIN_0=%x\r\n", bflb_gpio_read(gpio, GPIO_PIN_0));
    return bflb_gpio_read(gpio, GPIO_PIN_0);
}

uint8_t Right_Irtracking_Get(void)
{
	printf("[巡线]GPIO_PIN_1=%x\r\n", bflb_gpio_read(gpio, GPIO_PIN_1));
    return bflb_gpio_read(gpio, GPIO_PIN_1);
}
uint8_t Left_Irobstacle_Get(void)
{
	printf("[避障]GPIO_PIN_2=%x\r\n", bflb_gpio_read(gpio, GPIO_PIN_2));
    return bflb_gpio_read(gpio, GPIO_PIN_2);
}

uint8_t Right_Irobstacle_Get(void)
{
	printf("[避障]GPIO_PIN_3=%x\r\n", bflb_gpio_read(gpio, GPIO_PIN_3));
    return bflb_gpio_read(gpio, GPIO_PIN_3);
}

void robot_speed(int16_t left1_speed,int16_t left2_speed,int16_t right1_speed,int16_t right2_speed)
{	
    printf("Robot speed is %ld %ld\n",left1_speed, right1_speed);
    //bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH1, 0, left2_speed); /* duty = (500-100)/1000 = 40% */
    bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH3, 0, left1_speed); /* duty = (400-200)/1000 = 20% */
    //bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH2, 0, right2_speed); /* duty = (999-100)/1000 = 89.9% */
    bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH0, 0, right1_speed); /* duty = (500-0)/1000 = 50% */
    //bflb_pwm_v2_channel_positive_start(pwm, PWM_CH1);
    //bflb_pwm_v2_channel_positive_start(pwm, PWM_CH2);
    //bflb_pwm_v2_channel_positive_start(pwm, PWM_CH3);
    //bflb_pwm_v2_start(pwm);

}


// 基本的运动函数
// 机器人前进
void makerobo_run(int16_t speed,uint16_t time)  //前进函数
{
    printf("前进的速度是%d\n",speed);
    if(speed > 100)
    {
        speed = 400;
    }
    if(speed < 0)
    {
        speed = 0;
    }
    robot_speed(speed,0,speed,0);
    //bflb_mtimer_delay_ms(time);               // 时间为毫秒
}

void makerobo_brake(uint16_t time) //刹车函数
{
	robot_speed(0,0,0,0);     // 电机停止 
	//bflb_mtimer_delay_ms(time);        // 时间为毫秒   
}

void makerobo_Left(int16_t speed,uint16_t time) //左转函数
{
	    if(speed > 100)
        {
            speed = 400;
        }
        if(speed < 0)
        {
            speed = 0;
        }
		robot_speed(speed,0,0,0);
		//bflb_mtimer_delay_ms(time);            //时间为毫秒  
	  //robot_speed(0,0,0,0);           // 机器人停止

}

void makerobo_Spin_Left(int16_t speed,uint16_t time) //左旋转函数(原地旋转)
{
	if(speed > 100)
    {
        speed = 400;
    }
    if(speed < 0)
    {
        speed = 0;
    }  
    robot_speed(speed,0,0,speed);
    //bflb_mtimer_delay_ms(time);                 //时间为毫秒 
    //robot_speed(0,0,0,0);           // 机器人停止			
}

void makerobo_Right(int16_t speed,uint16_t time) //右转函数
{
	if(speed > 100)
    {
        speed = 400;
    }
    if(speed < 0)
    {
        speed = 0;
    }
	robot_speed(0,0,speed,0);
	//bflb_mtimer_delay_ms(time);            //时间为毫秒  
	//robot_speed(0,0,0,0);           // 机器人停止

}

void makerobo_Spin_Right(int16_t speed,uint16_t time) //右旋转函数
{
	if(speed > 100)
    {
        speed = 400;
    }
    if(speed < 0)
    {
        speed = 0;
    }  
	robot_speed(0,speed,speed,0);
	//bflb_mtimer_delay_ms(time);               //时间为毫秒 
    //robot_speed(0,0,0,0);           // 机器人停止			
}

void makerobo_back(int16_t speed,uint16_t time)  //后退函数
{
    if(speed > 100)
    {
        speed = 400;
    }
    if(speed < 0)
    {
        speed = 0;
    }
	robot_speed(0,speed,0,speed);
    //bflb_mtimer_delay_ms(time);          // 时间为毫秒
	//robot_speed(0,0,0,0);           // 机器人停止
 
}

// 整合循迹和避障的主控制函数
void Robot_Control()
{
    // 读取传感器状态
    uint8_t left_obstacle = Left_Irobstacle_Get();
    uint8_t right_obstacle = Right_Irobstacle_Get();
    uint8_t left_track = Left_Irtracking_Get();
    uint8_t right_track = Right_Irtracking_Get();
    
    // 避障检测（高优先级）
    // if (left_obstacle == 0 || right_obstacle == 0) {
    //     printf("[模式]避障\n");
    //     current_mode = MODE_OBSTACLE_AVOID;
    //     avoidance_start_time = bflb_mtimer_get_time_ms();
        
    //     if (left_obstacle == 0 && right_obstacle == 0) {
    //         // 前方有障碍物，后退并转向
    //         makerobo_back(300, 0);
    //     } 
    //     else if (left_obstacle == 0) {
    //         // 左侧有障碍物，向右转
    //         makerobo_Spin_Right(300, 0);
    //     } 
    //     else {
    //         // 右侧有障碍物，向左转
    //         makerobo_Spin_Left(300, 0);
    //     }
    //     return;
    // }
    
    // // 如果正在避障且未超时，保持避障动作
    // if (current_mode == MODE_OBSTACLE_AVOID) {
    //     uint32_t current_time = bflb_mtimer_get_time_ms();
    //     if (current_time - avoidance_start_time < AVOIDANCE_DURATION) {
    //         return;
    //     }
    // }
    
    // 无障碍物时执行循迹
    current_mode = MODE_LINE_TRACKING;
    printf("[模式]循迹\n");
    
    if (left_track == 0 && right_track == 0) {
        printf("[循迹]前进\n");
        makerobo_run(450, 0);
    }
    else if (left_track == 1 && right_track == 0) {
        printf("[循迹]左转\n");
        makerobo_Left(450, 0);
    }
    else if (left_track == 0 && right_track == 1) {
        printf("[循迹]右转\n");
        makerobo_Right(450, 0);
    }
    else {
        printf("[循迹]停止\n");
        makerobo_brake(0);
    }
}

int main(void)
{
    board_init();
    board_pwm_gpio_init();

    pwm = bflb_device_get_by_name("pwm_v2_0");
    gpio = bflb_device_get_by_name("gpio");

    /* period = .PBCLK / .clk_div / .period = 80MHz / 80 / 1000 = 1KHz */
    struct bflb_pwm_v2_config_s cfg = {
        .clk_source = BFLB_SYSTEM_PBCLK,
        .clk_div = 80,
        .period = 1000,
    };

    bflb_pwm_v2_init(pwm, &cfg);

    Irtracking_Init();

    while (1)
    {
        Robot_Control();
        bflb_mtimer_delay_ms(20); 
        //Left_Irtracking_Get();
        //Right_Irtracking_Get();
    }
}
