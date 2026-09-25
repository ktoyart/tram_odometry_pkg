FROM ros:humble-ros-base

# Устанавливаем зависимости системы
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем питоновские зависимости
RUN pip3 install setuptools numpy pyproj

# Создаем структуру ROS 2 workspace
WORKDIR /ros2_ws
RUN mkdir src

# Копируем пакет сообщений (если он нужен для сборки, предполагается что он рядом)
# COPY tram_vehicle_msgs src/tram_vehicle_msgs

# Копируем наше решение
COPY . src/tram_odometry_pkg/

# Собираем workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Настраиваем entrypoint
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run tram_odometry_pkg odometry_node"]
