FROM ros:humble-ros-base

ENV PATH="$HOME/.local/bin:$PATH"

RUN apt update && apt install -y \
        curl

RUN curl -sSL https://install.python-poetry.org | python3 -

RUN git clone https://github.com/RobotecAI/rai.git

WORKDIR /rai

RUN /root/.local/bin/poetry install

COPY src/ src/

RUN rosdep install --from-paths src --ignore-src -r -y

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install

# RUN export PATH="$HOME/.local/bin:$PATH" && \
#     source ./setup_shell.sh

COPY ./ros_entrypoint.sh /