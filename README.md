# **Descripción del Proyecto**

Este proyecto se centra en el diseño y control de una muñeca esférica de 3 grados de libertad (GDL) para un brazo delta. El objetivo es proporcionar a este brazo los grados de libertad necesarios para controlar su orientación. Este repositorio contiene el código de control de motores y otras funcionalidades relacionadas con la muñeca esférica, mientras que la parte de desarrollo 3D y experimentación se encuentra separada en otros repositorios o documentos.

## **Tabla de Contenidos**

- [Instalación](#instalación)
- [Uso](#uso)
- [Características](#características)
- [Contribuyendo](#contribuyendo)
- [Licencia](#licencia)
- [Contacto](#contacto)

## **Instalación**

Para comenzar a usar este proyecto, sigue estos pasos:

1. **Instalar la librería de motores Dynamixel SDK:**  
   Sigue las instrucciones detalladas en el siguiente enlace para instalar la librería necesaria:  
   [Dynamixel ROS Library](https://github.com/TaISLab/dynamixel_ros_library.git).

2. **Clonar el repositorio:**  
   Clona este repositorio para obtener el código:
   git clone https://github.com/Robotics-Mechatronics-UMA/spherical_wrist_delta.git

4. Compilar las dependencias:
   Después de clonar el repositorio, compila las dependencias usando el comando:
   catkin_make

6. Cargar el archivo de configuración:
   Carga el setup file para configurar el entorno de trabajo:
   source devel/setup.bash

## **Uso**

Para usar el paquete en tu proyecto, sigue estos pasos:

Incluye el paquete en los archivos CMakeLists.txt y package.xml de tu proyecto.

Después de hacer los cambios necesarios, podrás usar las funcionalidades implementadas en este paquete. Este paquete se integra con otros módulos de control para proporcionar una solución completa para el control de la muñeca esférica.

## **Características**

El paquete es compatible con cualquier muñeca esférica que utilice motores Dynamixel, siempre que se siga el formato de orientación Roll-Pitch-Yaw.
Incluye dos nodos ejecutables:
Un nodo para controlar la orientación de la muñeca, que publica los ángulos a través de un topic.
Un nodo que utiliza una IMU para simular los datos de orientación de un dron, útil para pruebas y simulaciones.

## Contribuyentes

## Licencia

## **Contacto**

Si tienes alguna pregunta o deseas más información sobre el proyecto, no dudes en contactarme:

Correo electrónico: [ordavidro@gmail.com](mailto:ordavidro@gmail.com) 
