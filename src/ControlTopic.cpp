/**
 * Este ejemplo demuestra cómo usar la biblioteca con ROS.
 * En este caso, un nodo de ROS es responsable de verificar permanentemente un tema 'user_position_input'
 * y cambiar la posición de los DMXLs si hay alguna información en el tema.
 * 
 * 1. Iniciar roscore:
 *      roscore
 * 
 * 2. Iniciar testPositionControl:
 *      rosrun dynamixel_ros_library ControlTopic
 * 
 * 3. Publicar cualquier posición entre 0 y 360 grados en el tema creado:
 *      rostopic pub -1 /user_position_input std_msgs/Int32MultiArray "data: [-, -, -]"
 */

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <dynamixel_ros_library.h>

// Declarar motores
dynamixelMotor motor1;
dynamixelMotor motor2;
dynamixelMotor motor3;

void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    // Verificar que el tamaño del array de posiciones sea válido
    if (msg->data.size() != 3) {
        ROS_ERROR("Received position array of invalid size, expected size 3");
        return;
    }

    int position1 = msg->data[0];
    int position2 = msg->data[1];
    int position3 = msg->data[2];

    // Motor 1
    if (position1 >= 0 && position1 <= 360) {
        if (motor1.getOperatingMode() != "Position Control") {
            if (motor1.getTorqueState()) {
                motor1.setTorqueState(false);
            }
            motor1.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
        }
        motor1.setTorqueState(true);
        motor1.setGoalPosition(position1);
    }

    // Motor 2
    if (position2 >= 0 && position2 <= 360) {
        if (motor2.getOperatingMode() != "Position Control") {
            if (motor2.getTorqueState()) {
                motor2.setTorqueState(false);
            }
            motor2.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
        }
        motor2.setTorqueState(true);
        motor2.setGoalPosition(position2);
    }

    // Motor 3
    if (position3 >= 0 && position3 <= 360) {
        if (motor3.getOperatingMode() != "Position Control") {
            if (motor3.getTorqueState()) {
                motor3.setTorqueState(false);
            }
            motor3.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
        }
        motor3.setTorqueState(true);
        motor3.setGoalPosition(position3);
    }
}

// Función para publicar la posición de los motores
void publishPosition(ros::Publisher& pub) {
    // Publicar las posiciones actuales de los motores
    std_msgs::Int32MultiArray pos_msg;
    pos_msg.data.push_back(motor1.getPresentPosition());
    pos_msg.data.push_back(motor2.getPresentPosition());
    pos_msg.data.push_back(motor3.getPresentPosition());

    pub.publish(pos_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrist_orientation_node");
    ros::NodeHandle n;

    char* port_name = "/dev/ttyUSB0"; 
    int dmxl_id_0 = 1; 
    int dmxl_id_1 = 2; 
    int dmxl_id_2 = 3; 

    float protocol_version = 2.0;
    int baud_rate =  57600;

    // Inicializar motor 1
    motor1 = dynamixelMotor("motor1", dmxl_id_0);
    if (!dynamixelMotor::iniComm(port_name, protocol_version, baud_rate)) {
        ROS_ERROR("Failed to initialize communication for motor1");
        return -1;
    }
    motor1.setControlTable();

    // Inicializar motor 2
    motor2 = dynamixelMotor("motor2", dmxl_id_1);
    if (!dynamixelMotor::iniComm(port_name, protocol_version, baud_rate)) {
        ROS_ERROR("Failed to initialize communication for motor2");
        return -1;
    }
    motor2.setControlTable();

    // Inicializar motor 3
    motor3 = dynamixelMotor("motor3", dmxl_id_2);
    if (!dynamixelMotor::iniComm(port_name, protocol_version, baud_rate)) {
        ROS_ERROR("Failed to initialize communication for motor3");
        return -1;
    }
    motor3.setControlTable();

    // Suscribirse al topic para recibir las posiciones
    ros::Subscriber sub1 = n.subscribe("user_position_input", 10, positionCallback);

    // Crear un publicador para publicar la posición de los motores
    ros::Publisher position_pub = n.advertise<std_msgs::Int32MultiArray>("motor_positions", 10);

    // Mantener el nodo en ejecución y publicar las posiciones periódicamente
    ros::Rate loop_rate(10); // Frecuencia de publicación
    while (ros::ok()) {
        ros::spinOnce(); // Procesa las callbacks una vez

        // Publicar la posición de los motores
        publishPosition(position_pub);

        loop_rate.sleep(); // Controlar la frecuencia de publicación
    }

    return 0;
}
