#include <Arduino.h>
#include <RosController.h>
#include <RobotBase.h>
#include <ImuBase.h>
#include <Imu.h>
#include <RosAdapterRobot.h>
#include <RosAdapterImu.h>
#include <RobotFactoryBLDC.h>
#include <RosConfigBLDC.h>
#include <TaskScheduler.h>
#include <bldc_interface_uart.h>
#include <comm_uart.h>

//----Global variables
Scheduler runner;
RosController  * ros_controller = 0;
RobotBase * robot = 0;
ImuBase * imu = 0;
int wheel_current = 0;


//¿TODO IT IS NECESSARY TO USE ENCODER CLASS OR THE WHEEL CLASS IS ENOUGH?
void bldc_run_timer_callback();
void bldc_read_serial_callback();
void bldc_read_values_callback();
void ros_callback();

void bldc_val_received_1(mc_values *val); 
void bldc_val_received_2(mc_values *val);

Task bldc_run_timer_task(1, TASK_FOREVER, &bldc_run_timer_callback);
Task bldc_read_serial_task(0, TASK_FOREVER, &bldc_read_serial_callback);
Task bldc_read_values_task(250, TASK_FOREVER, &bldc_read_values_callback);
Task ros_task(20, TASK_FOREVER, &ros_callback);

/**
 * 
 */
void bldc_run_timer_callback() 
{
	bldc_interface_uart_run_timer();
}

/**
 * 
 */
void bldc_read_serial_callback() { 
  while (Serial1.available()) {
	  bldc_interface_uart_process_byte(Serial1.read());
	}
}

/**
 * 
 */
void bldc_read_values_callback() {   
  if (wheel_current == 0) 
  {
    wheel_current = 1;
    bldc_interface_set_rx_value_func(bldc_val_received_2);
  }
  else
  {    
    wheel_current = 0;
    bldc_interface_set_rx_value_func(bldc_val_received_1);
  }

  WheelBase * wheel = robot->getWheel(wheel_current);
  BLCDHardwareController * controller = (BLCDHardwareController *)(wheel->getHardwareController());
  
  bldc_interface_set_forward_can(controller->getCANId()); //TODO
  bldc_interface_get_values();
}

void bldc_val_received_1(mc_values *val) 
{
  WheelBLCD * wheel = (WheelBLCD*)robot->getWheel(0);
  BLCDHardwareController * controller = (BLCDHardwareController *)(wheel->getHardwareController());

  //TODO Add this parameters in the configuration file
  float vel = ( (val->rpm/15.0) * (2*PI)/60 * controller->invert_ );  //radians per second

  wheel->setCurrentVelocity( vel ) ;
  //TODO WHY IS THE DISTANCE NEEDED FOR?
  wheel->setDistance( ( val->tachometer/90.0) * 2 * PI * 0.125 * controller->invert_ ); //meters

  bldc_read_values_task.restart();
}

void bldc_val_received_2(mc_values *val) 
{
  WheelBLCD * wheel = (WheelBLCD*)robot->getWheel(1);
  BLCDHardwareController * controller = (BLCDHardwareController *)(wheel->getHardwareController());

  //TODO Add this parameters in the configuration file
  float vel = ( (val->rpm/15.0) * (2*PI)/60 * controller->invert_ );  //radians per second

  wheel->setCurrentVelocity( vel ) ;
  wheel->setDistance( ( val->tachometer/90.0) * 2 * PI * 0.125 * controller->invert_ ); //meters

  bldc_read_values_task.restart();
}

void setup() {
  Serial.begin(9600);
  delay(5000);

  Serial.println("Gara Controller v2.0");

  RosAdapterRobot * ros_adapter_robot = new RosAdapterRobot();
  RosAdapterImu * ros_adapter_imu = new RosAdapterImu();

  ros_controller = new RosController();
  ros_controller->addNode(ros_adapter_robot);
  ros_controller->addNode(ros_adapter_imu);
  ros_controller->init();

  RosConfigBLDC * ros_config_motor = new RosConfigBLDC();
  ros_controller->readConfiguration(ros_config_motor);

  RobotFactory * factory = new  RobotFactoryBLDC(ros_config_motor); 
  robot =  factory->assembly();

  imu = new Imu();
  
  comm_uart_init();

  delay(1000);  //TODO FIND OUT WHY IS NECESSARY

  runner.init();

  runner.addTask(bldc_read_values_task);
  Serial.println("BLDC read values task created");

  runner.addTask(bldc_run_timer_task);
  Serial.println("BLCD uart run time task created");

  runner.addTask(bldc_read_serial_task);
  Serial.println("BLCD read serial task created");

	runner.addTask(ros_task);
  Serial.println("ROS task created");

  delay(5000);

  Serial.println("BLDC read values task  enabled");
 	bldc_read_values_task.enable();

  Serial.println("BLCD uart run time task enabled");
 	bldc_run_timer_task.enable();
 
	Serial.println("BLCD read serial task enabled");
  bldc_read_serial_task.enable();

  Serial.println("ROS task  enabled");
  ros_task.enable();

  ros_adapter_robot->attachRobot(robot);
  ros_adapter_imu->attachImu(imu);

}

void ros_callback() 
{
  if (ros_controller != 0)
  {
    ros_controller->update();
  }

  if (robot != 0) 
  {
    robot->update(0.25); 
  }
}

void loop() 
{
  runner.execute();
}