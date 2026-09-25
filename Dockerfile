FROM ros:humble-ros-base

# Устанавливаем зависимости системы
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем питоновские зависимости
RUN pip3 install setuptools "numpy>=2.0.0" pyproj

# Создаем структуру ROS 2 workspace
WORKDIR /ros2_ws
RUN mkdir src

# Копируем пакет сообщений, который должен лежать рядом
COPY tram_vehicle_msgs src/tram_vehicle_msgs

# Копируем наше решение
COPY . src/tram_odometry_pkg/

# Удаляем вложенную копию сообщений из пакета решения (если она случайно скопировалась как точка)
RUN rm -rf src/tram_odometry_pkg/tram_vehicle_msgs

# Собираем workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Настраиваем entrypoint
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run tram_odometry_pkg odometry_node"]
