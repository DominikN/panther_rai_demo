# syntax=docker/dockerfile:1.10.0
# https://docs.docker.com/build/buildkit/dockerfile-release-notes/

FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
        curl

RUN curl -sSL https://install.python-poetry.org | python3 -

RUN git clone https://github.com/RobotecAI/rai.git

WORKDIR /rai

RUN export PATH="$HOME/.local/bin:$PATH" && \
    poetry install && \
    rosdep install --from-paths src --ignore-src -r -y

# build RAI
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install

# build Whoami
COPY src/ src/

RUN --mount=type=secret,id=OPENAI_API_KEY,env=OPENAI_API_KEY \
    export PATH="$HOME/.local/bin:$PATH" && \
    source ./setup_shell.sh && \
    yes | poetry run parse_whoami_package src/panther_whoami/description

RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install

# # RUN export PATH="$HOME/.local/bin:$PATH" && \
# #     source ./setup_shell.sh

RUN apt update && apt install -y \
        ros-$ROS_DISTRO-opennav-docking-msgs

COPY ./ros_entrypoint.sh /