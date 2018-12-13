#include <Arduino.h>
#include <RosController.h>
#include <RobotBase.h>
#include <RosAdapterRobot.h>
#include <RobotFactoryBLDC.h>
#include <RosConfigBLDC.h>
#include <TaskScheduler.h>
#include <bldc_interface_uart.h>

//Global variables
RosController  * ros_controller = 0;
RobotBase * robot = 0;
Scheduler runner;


//TODO FIND OUT TO RUN THIS TASK IN THE LOWER LEVER CLASS : ENCODER.
//¿TODO IT IS NECESSARY TO USE ENCODER CLASS OR THE WHEEL CLASS IS ENOUGH?
void bldc_run_timer_callback();
void bldc_read_serial_callback();
void bldc_read_values_callback();

void ros_callback();

Task bldc_run_timer_task(1000, TASK_FOREVER, &bldc_run_timer_callback);
Task bldc_read_serial_task(0, TASK_FOREVER, &bldc_read_serial_callback);
Task bldc_read_values_task(5000, 10000, &bldc_read_values_callback);
Task ros_task(100, TASK_FOREVER, &ros_callback);

void setup() {

  Serial.begin(9600);

  RosAdapterRobot * ros_adapter_robot = new RosAdapterRobot();

  ros_controller = new RosController();
  ros_controller->addNode(ros_adapter_robot);
  ros_controller->init();

  RosConfigBLDC * ros_config_motor = new RosConfigBLDC();
  ros_controller->readConfiguration(ros_config_motor);

  RobotFactory * factory = new  RobotFactoryBLDC(ros_config_motor); 
  robot =  factory->assembly();
  ros_adapter_robot->attachRobot(robot);

  runner.init();

  runner.addTask(bldc_run_timer_task);
  Serial.println("BLCD uart run time task created");

  runner.addTask(bldc_read_values_task);
  Serial.println("BLDC read values task created");

  runner.addTask(bldc_read_serial_task);
  Serial.println("BLCD read serial task created");

	runner.addTask(ros_task);
  Serial.println("ROS task created");

  Serial.println("BLCD uart run time task enabled");
 	//bldc_run_timer_task.enable();

  Serial.println("BLDC read values task  enabled");
 	//bldc_read_values_task.enable();
  
	Serial.println("BLCD read serial task enabled");
  //bldc_read_serial_task.enable();

  Serial.println("ROS task  enabled");
  ros_task.enable();
}

void bldc_run_timer_callback() 
{
  //Serial.println("bldc_interface_uart_run_timer_callback: ");
	bldc_interface_uart_run_timer();
}

void bldc_read_serial_callback() {
  //Serial.print("bldc_interface_read_serial_callback: ");
  while (Serial1.available()) {
	  bldc_interface_uart_process_byte(Serial1.read());
	}
}

void bldc_read_values_callback() {
  //Serial.println("bldc_interface_read_values_callback: ");
  bldc_interface_set_forward_can(0); //TODO
  bldc_interface_get_values();
}

void bldc_val_received(mc_values *val) {
  //Serial.printf(F("RPM:           %.1f RPM\r\n"), val->rpm);
  bldc_read_values_task.restart();
}

void ros_callback() 
{

  if (ros_controller != 0)
  {
    ros_controller->update();
  }

  if (robot != 0) 
  {
    robot->update(0.1); 
  }

}

void loop() 
{
  runner.execute();
}