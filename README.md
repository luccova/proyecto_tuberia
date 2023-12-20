# Detector de fugas
Este proyecto fue realizado en el contexto del curso Duckietown del año 2023 en la Universidad de Chile
# Requerimientos
  Requerimientos de Hardware:
Duckiebot Mark 3
RPLIDAR A1M8
Joystick para controlar el bot

  Requerimientos de software (información compartida por colegas): 
imu_tools: https://github.com/CCNYRoboticsLab/imu_tools.git
mpu6050_driver: https://github.com/Brazilian-Institute-of-Robotics/mpu6050_driver.git
rplidar_ros: https://github.com/Slamtec/rplidar_ros.git
robot_localization: https://github.com/ros-perception/slam_gmapping.git
gmapping: https://github.com/cra-ros-pkg/robot_localization.git
map_server y amcl: https://github.com/ros-planning/navigation.git

# Iniciar los programas
especificar donde lanzamos cada cosa y donde la lanzamos
# Ver Odometría
En otra terminal lanzar rviz explicar como configurar el rviz incluir fotos y el gif del rviz
# Grabar rosbag
explicar como se graba el rosbag
# lanzar el detector de circulos junto al rosbag
explicar como se hace, incluir las fotos de los circulos encontrando fallas.
# hablar de motivaciones
El objetivo final de este proyecto era detectar fallas en tuberías ocupando un Lidar. La idea era recrear la tubería en Rviz utilizando las mediciones del Lidar y un sistema de odometría donde el Lidar al trabajar en 2D correspondería con las dimensiones del círculo en los ejex XY y la odometría ayudaría a indicar cuanto se había movido el robot en el eje Z.

Para implementar la odometría, primero calibramos la velocidad de las ruedas, pues en un inicio una avanzaba más que la otra. Luego de esto, corrimos el bot varias veces, midiendo la distancia recorrida y el tiempo empleado para así encontrar mediante regresión lineal la velocidad a la que avanzaban las ruedas. Una vez hecho esto, pudimos implementar el módulo de odometría, donde agregamos también un botón para reiniciar la cuenta de la odometría por si se querían tomar las muestras nuevamente.

Para implementar el Lidar, tuvimos que descargar los paquetes necesarios y aprender a utilizar Rviz para ver lo que detectaba el sensor. Con respecto a la detección de círculos, decidimos que íbamos a determinar la ubicación de las fallas con un programa que leía un rosbag ya grabado previamente para no consumir toda la memoria del bot. Para determinar cuales puntos estaban dentro y fuera de la circunferencia utilizamos una regresión.

Juntando estas dos partes es que pudimos completar satisfactoriamente el proyecto, la única cosa que por temas de tiempo no alcanzamos a hacer pero simplifica mucho el uso de este programa es el roslaunch que active todos los sistemas en conjunto, pues como pueden apreciar en el resto del texto, estos se deben correr por separado y en distintas terminales, lo que no es muy cómodo ni productivo.

#Agradecimientos

Estamos muy agradecidos de haber tenido la oportunidad de desarrollar este proyecto, queremos por ello, dar las gracias a toda la gente de Duckietown Engineering y especialmente a la de Maquintel por hacer de esta una increíble experiencia.
