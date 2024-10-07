#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <dynamixel_ros_library.h>

// Declarar motores
dynamixelMotor motor1;
dynamixelMotor motor2;
dynamixelMotor motor3;

ros::Publisher position_pub; // Declaración global

// Función para publicar la posición de los motores
void publishPosition() {
    // Publicar las posiciones actuales de los motores
    std_msgs::Int32MultiArray pos_msg;
    pos_msg.data.push_back(motor1.getPresentPosition());
    pos_msg.data.push_back(motor2.getPresentPosition());
    pos_msg.data.push_back(motor3.getPresentPosition());

    position_pub.publish(pos_msg); // Usar la variable global
}

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

    publishPosition(); // Publicar la posición
}

// Función para inicializar motores
bool initializeMotor(dynamixelMotor& motor, const char* port_name, int id) {
    motor = dynamixelMotor("motor", id);
    if (!dynamixelMotor::iniComm(const_cast<char*>(port_name), 2.0, 57600)) { // Uso de const_cast
        ROS_ERROR("Failed to initialize communication for motor with ID %d", id);
        return false;
    }
    motor.setControlTable();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrist_orientation_node");
    ros::NodeHandle n;

    const char* port_name = "/dev/ttyUSB2"; // Aquí no necesitas cambiar nada.
    int dmxl_id_0 = 1; 
    int dmxl_id_1 = 2; 
    int dmxl_id_2 = 3; 

    // Inicializar motores
    if (!initializeMotor(motor1, port_name, dmxl_id_0) || 
        !initializeMotor(motor2, port_name, dmxl_id_1) || 
        !initializeMotor(motor3, port_name, dmxl_id_2)) {
        return -1;
    }

    // Suscribirse al topic para recibir las posiciones
    ros::Subscriber sub1 = n.subscribe("user_position_input", 10, positionCallback);

    // Crear un publicador para publicar la posición de los motores
    position_pub = n.advertise<std_msgs::Int32MultiArray>("motor_positions", 10); // Inicialización correcta

    ros::spin(); // Iniciar el bucle de ROS

    return 0;
}
