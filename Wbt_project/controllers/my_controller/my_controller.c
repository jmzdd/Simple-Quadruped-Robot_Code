/*
 * File:          my_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h> 
//引入节点头文件，目的是为了能够调用robot节点的函数，例如：wb_robot_init();

#include <math.h>
#include <stdio.h>
//可引入C标准库

#include <webots/motor.h>
//同上robot头文件作用
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {

  /* necessary to initialize webots stuff */
  wb_robot_init();
  //此函数用于初始化控制器和Webots之间的通信。
  double time_step = wb_robot_get_basic_time_step();
  //返回 WorldInfo 节点的 basicTimeStep 字段的值
  
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   //电机命名变量初始化
  WbDeviceTag L1motor1 = wb_robot_get_device("left_1 motor1");
  WbDeviceTag L1motor2 = wb_robot_get_device("left_1 motor2");
  WbDeviceTag L2motor1 = wb_robot_get_device("left_2 motor1");
  WbDeviceTag L2motor2 = wb_robot_get_device("left_2 motor2");
  WbDeviceTag R1motor1 = wb_robot_get_device("Right_1 motor1");
  WbDeviceTag R1motor2 = wb_robot_get_device("Right_1 motor2");
  WbDeviceTag R2motor1 = wb_robot_get_device("Right_2 motor1");
  WbDeviceTag R2motor2 = wb_robot_get_device("Right_2 motor2");
  
  //设置电机速度

  //新建变量以存储所创建的机器人设备
  while (wb_robot_step(time_step) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     * double val = wb_distance_sensor_get_value(my_sensor);
     */
    //电机位置初始化
    //左脚1
    wb_motor_set_position(L1motor1, 1);
    wb_motor_set_position(L1motor2, 0);
    wb_robot_step(TIME_STEP);
    // //左脚2
    wb_motor_set_position(L2motor1, -0.5);
    wb_motor_set_position(L2motor2, 0);
    wb_robot_step(TIME_STEP);
    // //右脚1
    wb_motor_set_position(R1motor2, 0.5);
    wb_robot_step(TIME_STEP);
    // //右脚2
    wb_motor_set_position(R2motor2, 0.5);
    wb_robot_step(TIME_STEP);
    // wb_motor_set_position();属于电机功能部分api,是电机位置控制函数
    // //第一步-迈步
    wb_motor_set_position(R2motor2, -1);
    wb_motor_set_position(R2motor1, 1);
    wb_motor_set_position(R2motor2, -0.5);
    wb_robot_step(TIME_STEP);
    //第二步-迈步
    wb_motor_set_position(R1motor1, -0.5);
    wb_motor_set_position(R1motor2, 0);
    wb_motor_set_position(R2motor2, 0);
    wb_motor_set_position(L2motor1, 0.33);
    wb_motor_set_position(L2motor2, 0.5);
    wb_motor_set_position(L1motor2, -0.5);
    wb_robot_step(TIME_STEP);
    //第三步-移动
    wb_motor_set_position(L1motor2, -0.8);
    wb_motor_set_position(L1motor1, 0.33);
    wb_motor_set_position(L1motor2, 0.5);
    wb_robot_step(TIME_STEP);
    // //第四步-迈步
    wb_motor_set_position(L2motor2, -1);
    wb_motor_set_position(L2motor1, -0.5);
    wb_motor_set_position(L2motor2, -0.5);
    wb_robot_step(TIME_STEP);
    // //第五步-迈步
    wb_motor_set_position(R1motor2, -0.5);
    wb_motor_set_position(R2motor1, 0.33);
    wb_motor_set_position(R2motor2, 0.5);
    wb_motor_set_position(L2motor2, 0);
    wb_motor_set_position(L1motor1, 1);
    wb_motor_set_position(L1motor2, 0);
    wb_robot_step(TIME_STEP);
    //第六步-移动
    // wb_motor_set_position(L1motor1, 0.33);
    // wb_motor_set_position(L2motor2, -0.5);
    // wb_motor_set_position(R2motor1, 0.83);
    // wb_motor_set_position(L1motor2, 0.5);
    // wb_motor_set_position(R1motor2, 0);
    // wb_robot_step(TIME_STEP);
  /*
    counter++;
    if (counter == 50)
      wb_motor_set_position(motor1, 1);
    else if (counter == 100)
      wb_motor_set_position(motor1, -1);
    else if (counter == 200)
      wb_motor_set_position(motor1, 0);
    else if (counter == 300)
      wb_motor_set_position(motor2, 1);
    else if (counter == 400)
      wb_motor_set_position(motor2, -1);
    else if (counter > 500) {
      wb_motor_set_position(motor1, 0.5 * cos(0.01 * counter));
      wb_motor_set_position(motor2, 0.5 * sin(0.003 * counter));
    }
    */

    /* Process sensor data here */
    
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };
  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}