#include <Arduino.h>
#include <RosController.h>
#include <RobotBase.h>
#include <ImuBase.h>
#include <Imu.h>
#include <RosAdapterRobot.h>
#include <BLCDHardwareController.h>
#include <DifferentialWheeledRobot.h>
#include <WheelEncoder.h>
#include <RosAdapterImu.h>
#include <RosConfigBLDC.h>
#include <TaskScheduler.h>

//----Global variables
Scheduler                 runner;

RosController  *          ros_controller = 0;
RobotBase *               robot = 0;
WheelBase *               wheel1 = 0;
WheelBase *               wheel2 = 0;
BLCDHardwareController *  controller1 = 0;
BLCDHardwareController *  controller2 = 0;
ImuBase *                 imu = 0;
int                       wheel_current = 0;


//----Tasks
void                      clean_task_callback();
void                      fast_task_callback();
void                      medium_task_callback();
void                      slow_task_callback();

Task                      clean_task(1, TASK_FOREVER, &clean_task_callback);
Task                      fast_task(20, TASK_FOREVER, &fast_task_callback);
Task                      medium_task(100, TASK_FOREVER, &medium_task_callback);
Task                      slow_task(250, TASK_FOREVER, &slow_task_callback);

//----Setup
void setup() {

  Serial.begin(9600);

  delay(5000);

  Serial.println("Gara Controller v3.0");

  RosAdapterRobot * ros_adapter_robot = new RosAdapterRobot();
  RosAdapterImu * ros_adapter_imu = new RosAdapterImu();

  ros_controller = new RosController();
  ros_controller->addNode(ros_adapter_robot);
  ros_controller->addNode(ros_adapter_imu);
  ros_controller->init();

  RosConfigBLDC * ros_config_motor = new RosConfigBLDC();
  ros_controller->readConfiguration(ros_config_motor);

  RobotBase * robot = 
        new DifferentialWheeledRobot(ros_config_motor->robot_wheel_separation,
                                      ros_config_motor->robot_wheel_radious);

  controller1 = 
      new BLCDHardwareController(0,
                                  ros_config_motor->max_speed,
                                  ros_config_motor->min_duty,
                                  ros_config_motor->max_duty,
                                  ros_config_motor->wheel_config[0].can_id,
                                  ros_config_motor->wheel_config[0].invert);

  wheel1 = new WheelEncoder();
  wheel1->attachController(controller1);

  robot->addWheel(wheel1); 

  controller2 = 
      new BLCDHardwareController(1,
                                  ros_config_motor->max_speed,
                                  ros_config_motor->min_duty,
                                  ros_config_motor->max_duty,
                                  ros_config_motor->wheel_config[1].can_id,
                                  ros_config_motor->wheel_config[1].invert);                                  

  wheel2 = new WheelEncoder();
  wheel2->attachController(controller2);

  robot->addWheel(wheel2); 

  ros_adapter_robot->attachRobot(robot);
  
  imu = new Imu();

  ros_adapter_imu->attachImu(imu);
    
  delay(1000);  //TODO FIND OUT WHY IS NECESSARY

  runner.init();
    
  runner.addTask(clean_task);
  runner.addTask(fast_task);
  runner.addTask(medium_task);
  runner.addTask(slow_task);

  delay(5000); 

  clean_task.enable();
 	fast_task.enable();
 	medium_task.enable();
  slow_task.enable();
}

void clean_task_callback() 
{
  BLCDHardwareController::buffer_clean();
}

void fast_task_callback() 
{
  BLCDHardwareController * controller;
  
  controller = wheel_current == 0 ? controller1: controller2;
  wheel_current = wheel_current == 0 ? 1: 0;

  if (controller!=0)
    controller->update(0.05);
}

void medium_task_callback() 
{
  if (wheel1 != 0) 
    wheel1->update(0.01);
  if (wheel2 != 0) 
    wheel2->update(0.01);
  if (robot != 0) 
    robot->update(0.01); 
}

void slow_task_callback() 
{ 
  if (ros_controller != 0)
    ros_controller->update();
}

void loop() 
{
  runner.execute();
}