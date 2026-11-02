FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      libegl1 libgl1-mesa-dri python3-colcon-common-extensions python3-rosdep git \
    && rm -rf /var/lib/apt/lists/*

# Install package dependencies from the package.xml files.
COPY . /tmp/rb_sim_src
RUN apt-get update \
    && (rosdep init 2>/dev/null || true) && rosdep update \
    && rosdep install --from-paths /tmp/rb_sim_src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/* /tmp/rb_sim_src

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
