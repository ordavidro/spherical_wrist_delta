/**
 * This example is created to demonstrate how to use the library with ROS. 
 * In this case, a ROS node is responsible for checking permanently a 'user_position_input' topic
 * and to change the DMXLs position if there is some information in the topic.
 * 
 * 1. Launch roscore:
 *      roscore
 * 
 * 2. Launch PlotJuggler (only if you want to visualize some data):
 *      rosrun plotjuggler plotjuggler 
 * 
 * 3. Launch testPositionControl:
 *      rosrun dynamixel_ros_library testPositionControl  YOUR_PORT PROTOCOL_TYPE BAUDRATE DMXL_ID
 * 
 * 4. Publish any position between 0 and 360 degrees in the created topic:
 *      rostopic pub -1 /user_position_input std_msgs/Int32 "data: DEGREES"
*/
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <dynamixel_ros_library.h>
//#include "/home/ordavidro/wrist_ws/src/dynamixel_ros_library/package/dynamixel_ros_library/include/dynamixel_ros_library.h"
//#include "/home/ordavidro/wrist_ws/src/DynamixelSDK/ros/dynamixel_sdk/include/dynamixel_sdk/dynamixel_sdk.h"

dynamixelMotor motor1;
//  dynamixelMotor motor2;
//  //dynamixelMotor motor3;

//  void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
//  {
//      if (msg->data.size() != 3) {
//          ROS_ERROR("Received position array of invalid size, expected size 2");
//          return;
//      }

//      int position1 = msg->data[0];
//      int position2 = msg->data[1];
//      //int position3 = 0;//= msg->data[2];


//      // Motor 1
//      if(motor1.getOperatingMode() != "Position Control")
//      {
//          if(motor1.getTorqueState())
//          {
//              motor1.setTorqueState(false);
//          }
//          motor1.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
//      }
//      motor1.setTorqueState(true);
//      motor1.setGoalPosition(position1);

//      // Motor 2
//      if(motor2.getOperatingMode() != "Position Control")
//      {
//          if(motor2.getTorqueState())
//         {
//             motor2.setTorqueState(false);
//         }
//         motor2.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
//     }
//     motor2.setTorqueState(true);
//     motor2.setGoalPosition(position2);

// //     // Motor 3
// //     if(motor3.getOperatingMode() != "Position Control")
// //     {
// //         if(motor3.getTorqueState())
// //         {
// //             motor3.setTorqueState(false);
// //         }
// //         motor3.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
// //     }
// //     motor3.setTorqueState(true);
// //     motor3.setGoalPosition(position3);
// }


int main(int argc, char **argv)
{
    if (argc != 7) {
        ROS_ERROR("Usage: wrist_orientation_node port_name_0 dmxl_id_0 port_name_1 dmxl_id_1 protocol_version baud_rate");
        return -1;
    }

    char* port_name_0 = argv[1];   
    int dmxl_id_0 = atoi(argv[2]);
    char* port_name_1 = argv[3];
    int dmxl_id_1 = atoi(argv[4]);
    
    float protocol_version = atof(argv[5]);
    int baud_rate = atoi(argv[6]);

    // //dynamixelMotor motor1("motor1", dmxl_id_0);
    motor1 = dynamixelMotor("motor1", dmxl_id_0);
    dynamixelMotor::iniComm(port_name_0,protocol_version,baud_rate);
    // if (!dynamixelMotor::iniComm(port_name_0, protocol_version, baud_rate)) {
    //     ROS_ERROR("Failed to initialize communication for motor1");
    //     return -1;
    // }
    motor1.setControlTable();
    
    // motor2 = dynamixelMotor("motor2", dmxl_id_1);    
    // //dynamixelMotor::iniComm(port_name_1,protocol_version,baud_rate);
    // if (!dynamixelMotor::iniComm(port_name_1, protocol_version, baud_rate)) {
    //     ROS_ERROR("Failed to initialize communication for motor2");
    //     return -1;
    // }
    // motor2.setControlTable();
   
    ros::init(argc, argv, "wrist_orientation_node");
    ros::NodeHandle n;
    // Suscribirse al topic para recibir las posiciones
    // ros::Subscriber sub1 = n.subscribe("user_position_input", 10, positionCallback);
    
    ros::spin();

return 0;
}



