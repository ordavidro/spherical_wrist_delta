#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <dynamixel_ros_library.h>

// Variables para los motores
dynamixelMotor motor1;
dynamixelMotor motor2;
dynamixelMotor motor3;

int fixed_position = 280; // Posición fija del primer motor
int roll_position = 160;
int yaw_position = 195;
int pitch_position = 280;

// Callbacks para los topics de la IMU
void yawCallback(const std_msgs::Float32::ConstPtr& msg) {
    yaw_position = static_cast<int>(msg->data);
}

void rollCallback(const std_msgs::Float32::ConstPtr& msg) {
    roll_position = static_cast<int>(msg->data);
}

void pitchCallback(const std_msgs::Float32::ConstPtr& msg) {
    pitch_position = static_cast<int>(msg->data);
}

// Función para publicar la posición de los motores
void publishPosition(ros::Publisher& pub) {
    // Motor 2
    if (motor2.getOperatingMode() != "Position Control") {
        if (motor2.getTorqueState()) {
            motor2.setTorqueState(false);
        }
        motor2.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    }
    motor2.setTorqueState(true);
    motor2.setGoalPosition(pitch_position);

    // Motor 3
    if (motor3.getOperatingMode() != "Position Control") {
        if (motor3.getTorqueState()) {
            motor3.setTorqueState(false);
        }
        motor3.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    }
    motor3.setTorqueState(true);
    motor3.setGoalPosition(yaw_position);

    // Crear un mensaje para publicar las posiciones
    std_msgs::Int32MultiArray pos_msg;
    pos_msg.data.push_back(motor1.getPresentPosition());
    pos_msg.data.push_back(motor2.getPresentPosition());
    pos_msg.data.push_back(motor3.getPresentPosition());

    // Publicar el mensaje
    pub.publish(pos_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wrist_orientation_node");
    ros::NodeHandle n;

    char* port_name_motors = "/dev/ttyUSB2"; // Port name for motors
    int dmxl_id_0 = 1; // ID of the first motor
    int dmxl_id_1 = 2; // ID of the second motor
    int dmxl_id_2 = 3; // ID of the third motor

    float protocol_version = 2.0;
    int baud_rate = 57600;

    // Inicializar la comunicación para los motores
    if (!dynamixelMotor::iniComm(port_name_motors, protocol_version, baud_rate)) {
        ROS_ERROR("Failed to initialize communication for motors");
        return -1;
    }

    // Inicializar los motores
    motor1 = dynamixelMotor("motor1", dmxl_id_0);
    motor1.setControlTable();

    motor2 = dynamixelMotor("motor2", dmxl_id_1);
    motor2.setControlTable();

    motor3 = dynamixelMotor("motor3", dmxl_id_2);
    motor3.setControlTable();

    // Configurar los topics de la IMU para recibir los valores de yaw, roll y pitch
    ros::Subscriber sub_yaw = n.subscribe("/yaw_angle_topic", 10, yawCallback);
    ros::Subscriber sub_roll = n.subscribe("/roll_angle_topic", 10, rollCallback);
    ros::Subscriber sub_pitch = n.subscribe("/pitch_angle_topic", 10, pitchCallback);

    // Crear un publicador para publicar la posición de los motores
    ros::Publisher position_pub = n.advertise<std_msgs::Int32MultiArray>("motor_positions", 10);

    // Configurar motor 1 en la posición fija
    if (motor1.getOperatingMode() != "Position Control") {
        if (motor1.getTorqueState()) {
            motor1.setTorqueState(false);
        }
        motor1.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    }
    motor1.setTorqueState(true);
    motor1.setGoalPosition(fixed_position);

    ros::Rate loop_rate(10); // Frecuencia de publicación

    while (ros::ok()) {
        ros::spinOnce(); // Procesa las callbacks una vez
        publishPosition(position_pub); // Publica la posición de los motores basada en las últimas lecturas

        loop_rate.sleep(); // Controlar la frecuencia de publicación
    }

    return 0;
}
