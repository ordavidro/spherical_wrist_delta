/**
 * This example is created to demonstrate how to use the library with ROS. 
 * In this case, a ROS node is responsible for checking permanently a 'user_position_input' topic
 * and to change the DMXLs position if there is some information in the topic.
 * 
 * 1. Launch roscore:
 *      roscore
 * 
 * 2. Launch testPositionControl:
 *      rosrun dynamixel_ros_library ControlTopic
 * 
 * 3. Publish any position between 0 and 360 degrees in the created topic:
 *      
 *      rostopic pub -1 /user_position_input std_msgs/Int32MultiArray "data: [-, -, -]"
*/
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <dynamixel_ros_library.h>

dynamixelMotor motor1;
dynamixelMotor motor2;
dynamixelMotor motor3;

void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
 {
     if (msg->data.size() != 3) {
         ROS_ERROR("Received position array of invalid size, expected size 3");
         return;
     }

     int position1 = msg->data[0];
     int position2 = msg->data[1];
     int position3 = msg->data[2];


     // Motor 1
     if(motor1.getOperatingMode() != "Position Control")
     {
         if(motor1.getTorqueState())
         {
             motor1.setTorqueState(false);
         }
         motor1.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
     }
     motor1.setTorqueState(true);
     motor1.setGoalPosition(position1);

     // Motor 2
     if(motor2.getOperatingMode() != "Position Control")
     {
         if(motor2.getTorqueState())
        {
            motor2.setTorqueState(false);
        }
        motor2.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    }
    motor2.setTorqueState(true);
    motor2.setGoalPosition(position2);

     // Motor 3
     if(motor3.getOperatingMode() != "Position Control")
     {
         if(motor3.getTorqueState())
         {
             motor3.setTorqueState(false);
         }
         motor3.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
     }
     motor3.setTorqueState(true);
     motor3.setGoalPosition(position3);
}


int main(int argc, char **argv)
{
    //if (argc != 9) {
    //    ROS_ERROR("Usage: wrist_orientation_node port_name_50 dmxl_id_0 port_name_1 dmxl_id_1 protocol_version baud_rate");
    //    return -1;
    //}
    ros::init(argc, argv, "wrist_orientation_node");
    ros::NodeHandle n;

    char* port_name; //= "/dev/ttyUSB0"; //argv[1];   // "/dev/ttyUSB0";
    int dmxl_id_0 = 1; //atoi(argv[2]); //1;
    int dmxl_id_1 = 2; //atoi(argv[4]); //2;
    int dmxl_id_2 = 3; //atoi(argv[6]); // 3
    

    float protocol_version = 2.0;//atof(argv[7]); //2.0;
    int baud_rate =  57600; //atoi(argv[8]); //57600;


    motor1 = dynamixelMotor("motor1", dmxl_id_0);
     if (!dynamixelMotor::iniComm(port_name, protocol_version, baud_rate)) {
         ROS_ERROR("Failed to initialize communication for motor1");
         return -1;
     }
    motor1.setControlTable();
    
    
    motor2 = dynamixelMotor("motor2", dmxl_id_1);    
    if (!dynamixelMotor::iniComm(port_name, protocol_version, baud_rate)) {
        ROS_ERROR("Failed to initialize communication for motor2");
        return -1;
    }
    motor2.setControlTable();
   

    motor3 = dynamixelMotor("motor3", dmxl_id_2);    
    if (!dynamixelMotor::iniComm(port_name, protocol_version, baud_rate)) {
        ROS_ERROR("Failed to initialize communication for motor3");
        return -1;
    }
    motor3.setControlTable();

    
    // Suscribirse al topic para recibir las posiciones
    ros::Subscriber sub1 = n.subscribe("user_position_input", 10, positionCallback);
    
    ros::spin();

return 0;
}



