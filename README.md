# Nombre del Proyecto

Este proyecto se centra en el diseño y control de una muñeca de 3 grados de libertad (GDL) que proporciona a un brazo delta los grados de libertad necesarios para controlar su orientación. El código utilizado para controlar los motores se encuentra en el paquete `wrist_control`.

## Tabla de Contenidos

- [Instalación](#instalación)
- [Uso](#uso)
- [Características](#características)
- [Contribuyendo](#contribuyendo)
- [Licencia](#licencia)
- [Contacto](#contacto)

## Instalación

Hay que seguir los siguientes pasos:
1. Instalar la librería de los motores dynamixel SDK siguiendo los pasos descritos en el siguiente enlace https://github.com/TaISLab/dynamixel_ros_library.git

2. Clonar el repositorio actual:
git clone https: https://github.com/Robotics-Mechatronics-UMA/spherical_wrist_delta.git

3. Compilar las dependencias
catkin_make

4. Cargar el setup file:
   source devel/setup.bash

## Uso
Para usar la librería en el proyecto, incluyelo en los archivos CMakeList.txt y en el package.xml

Después de los cambios podemos usar las funcionalidade implementadas en la librería en nuestro proyecto. 

## Características
El paquete puede usarse con cualquier muñeca esférica que use motores dynamixel siempre y cuando se tenga en cuenta que debe de ser Roll-Pitch-Yaw.
El paquete incluye dos nodos ejecutables en el src. Uno el cuál sirve para controlar la orientación de la muñeca mediante la publicación de los ángulos mediante un topic y el otro utiliza una IMU para simular los datos de orientación de un dron.
## Contribuyentes

## Licencia

## Contacto
Si tienes alguna pregunta o deseas más información, no dudes en contactarme:

- David Rodríguez - [ordajuegos@gmail.com](mailto:ordajuegos@gmail.com) 
