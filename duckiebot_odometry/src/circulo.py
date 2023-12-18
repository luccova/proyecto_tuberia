import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import matplotlib.pyplot as plt

def ransac_circle_2d(data, angles, iterations=1000, threshold=0.1):
    best_circle = None
    best_inliers = 0

    for _ in range(iterations):
        # Paso 1: Muestreo aleatorio de 3 puntos
        sample_indices = np.random.choice(len(data), 3, replace=False)
        sample = data[sample_indices]

        # Paso 2: Estimacion del modelo (ecuacion del circulo)
        circle_params = fit_circle_2d(sample)

        # Paso 3: Determinar el conjunto de consenso (inliers)
        distances = np.abs(np.linalg.norm(data - circle_params[:2], axis=1) - circle_params[2])
        inliers = np.where(distances < threshold)[0]

        # Paso 4: Verificar si este modelo es el mejor hasta ahora
        if len(inliers) > best_inliers:
            best_inliers = len(inliers)
            best_circle = circle_params

    return best_circle

def fit_circle_2d(points):
    # Ajuste de un circulo 2D a partir de puntos usando OpenCV
    center, radius = cv2.minEnclosingCircle(points)
    return np.array([center[0], center[1], radius])

def find_points_outside_circle(points, circle_params, error_threshold=0.01):
    # Encontrar puntos fuera del circulo con un error de umbral
    distances = np.abs(np.linalg.norm(points - circle_params[:2], axis=1) - circle_params[2])
    outside_indices = np.where(distances > error_threshold)[0]
    return points[outside_indices]

def lidar_data_callback(msg):
    # Convertir el mensaje LaserScan a un arreglo numpy
    angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
    points = np.array([[msg.ranges[i] * np.cos(angles[i]), msg.ranges[i] * np.sin(angles[i])] for i in range(len(msg.ranges))], dtype=np.float32)

    # Aplicar RANSAC para estimar el circulo
    estimated_circle = ransac_circle_2d(points, angles)

    # Encontrar puntos fuera del circulo con un error de 1 cm
    outliers = find_points_outside_circle(points, estimated_circle, error_threshold=0.01)

    # Mostrar el resultado
    # print("Parametros del circulo estimado:", estimated_circle) #no queremos mostrar esto por ahora, se puede descomentar
    print("Puntos fuera del circulo con un error de 1 cm:", outliers)

    # Trazar los datos
    plt.figure(figsize=(8, 8))

    # Trazar los puntos de la nube de puntos
    plt.scatter(points[:, 0], points[:, 1], label='Puntos LIDAR', color='blue', alpha=0.5)

    # Trazar el circulo estimado
    circle = plt.Circle((estimated_circle[0], estimated_circle[1]), estimated_circle[2], edgecolor='r', facecolor='none', label='Circulo estimado')
    plt.gca().add_patch(circle)

    # Trazar los puntos fuera del circulo
    plt.scatter(outliers[:, 0], outliers[:, 1], label='Puntos fuera del circulo', color='red', marker='x')

    # Configuracion del grafico
    plt.title('Estimacion del circulo y puntos fuera del circulo')
    plt.xlabel('Coordenada X')
    plt.ylabel('Coordenada Y')
    plt.legend()
    plt.grid(True)
    plt.show()
if __name__ == "__main__":
    # Inicializar el nodo de ROS
    rospy.init_node("lidar_data_processing_node")

    # Suscribirse al topico que publica el nodo de LIDAR con la nube de puntos
    rospy.Subscriber("/scan", LaserScan, lidar_data_callback)  # Reemplazar con el nombre real del topico

    # Mantener el nodo en ejecucion
    rospy.spin()

