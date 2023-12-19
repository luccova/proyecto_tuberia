import rosbag
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

def ransac_circle_2d(data, angles, iterations=1000, threshold=0.01):
    best_circle = None
    best_inliers = 0

    for _ in range(iterations):
        # Paso 1: Muestreo aleatorio de 3 puntos
        sample_indices = np.random.choice(len(data), 3, replace=False)
        sample = data[sample_indices]

        # Paso 2: Estimación del modelo (ecuación del círculo)
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
    # Ajuste de un círculo 2D a partir de puntos usando OpenCV
    center, radius = cv2.minEnclosingCircle(points)
    return np.array([center[0], center[1], radius])
    
def find_points_outside_circle(points, circle_params, error_threshold=0.01):
    # Encontrar puntos fuera del círculo con un error de umbral
    distances = np.abs(np.linalg.norm(points - circle_params[:2], axis=1) - circle_params[2])
    outside_indices = np.where(distances > error_threshold)[0]
    return points[outside_indices]

if __name__ == "__main__":
    # Especificar el nombre del archivo rosbag
    bag_filename = "2023-12-12-21-01-18.bag"

    # Crear una lista para almacenar los datos
    lidar_data = []

    # Abrir el archivo rosbag
    with rosbag.Bag(bag_filename, "r") as bag:
        # Leer mensajes del tópico /lidar_data_topic (ajustar según el tópico real)
        for topic, msg, _ in bag.read_messages(topics=["/scan"]):
            # Convertir el mensaje LaserScan a un arreglo numpy
            angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            points = np.array([[msg.ranges[i] * np.cos(angles[i]), msg.ranges[i] * np.sin(angles[i])] for i in range(len(msg.ranges))], dtype=np.float32)

            # Agregar datos al conjunto
            lidar_data.append(points)

    # Convertir la lista a un array de NumPy
    lidar_data = np.vstack(lidar_data)

    # Aplicar RANSAC para estimar el círculo
    estimated_circle = ransac_circle_2d(lidar_data, angles)

    # Encontrar puntos fuera del círculo con un error de 1 cm
    outliers = find_points_outside_circle(lidar_data, estimated_circle, error_threshold=0.01)

    # Mostrar el resultado
    print("Parametros del circulo estimado:", estimated_circle)
    print("Puntos fuera del circulo con un error de 1 cm:", outliers)


if __name__ == "__main__":
    # Especificar el nombre del archivo rosbag
    bag_filename = "2023-12-12-21-01-18.bag"

    # Configuración inicial del gráfico
    plt.figure(figsize=(8, 8))
    plt.title('Estimacion del circulo y puntos fuera del circulo')
    plt.xlabel('Coordenada X')
    plt.ylabel('Coordenada Y')

    # Abrir el archivo rosbag
    with rosbag.Bag(bag_filename, "r") as bag:
        # Iterar sobre los mensajes del tópico /scan (ajustar según el tópico real)
        for topic, msg, _ in bag.read_messages(topics=["/scan"]):
            # Convertir el mensaje LaserScan a un arreglo numpy
            angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            points = np.array([[msg.ranges[i] * np.cos(angles[i]), msg.ranges[i] * np.sin(angles[i])] for i in range(len(msg.ranges))], dtype=np.float32)

            # Aplicar RANSAC para estimar el círculo
            estimated_circle = ransac_circle_2d(points, angles)

            # Encontrar puntos fuera del círculo con un error de 1 cm
            outliers = find_points_outside_circle(points, estimated_circle, error_threshold=0.01)

            # Borrar el gráfico anterior
            plt.clf()

            # Trazar los puntos de la nube de puntos
            plt.scatter(points[:, 0], points[:, 1], label='Puntos LIDAR', color='blue', alpha=0.5)

            # Trazar el círculo estimado
            circle = plt.Circle((estimated_circle[0], estimated_circle[1]), estimated_circle[2], edgecolor='r', facecolor='none', label='Circulo estimado')
            plt.gca().add_patch(circle)

            # Trazar los puntos fuera del círculo
            plt.scatter(outliers[:, 0], outliers[:, 1], label='Puntos fuera del circulo', color='red', marker='x')

            # Configuración del gráfico
            plt.legend()
            plt.grid(True)

            # Mostrar el gráfico
            plt.pause(1)  # Pausa para permitir la actualización del gráfico en tiempo real

            # Introducir una pausa de un segundo
            time.sleep(0.1)
