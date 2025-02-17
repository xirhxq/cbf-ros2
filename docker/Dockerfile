FROM osrf/mbzirc:mbzirc_sim_latest

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

USER root

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    sudo \
    vim \
    tmuxinator \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/ERGO-Code/HiGHS.git /tmp/HiGHS \
    && cd /tmp/HiGHS \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build build --parallel \
    && cd build \
    && ctest --output-on-failure \
    && cmake --install . \
    && cd / \
    && rm -rf /tmp/HiGHS \
    && ldconfig

RUN apt-get update && apt-get install -y \
    neofetch \
    && rm -rf /var/lib/apt/lists/*

RUN echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && mkdir -p /home/developer/cbf_ws/src \
    && chown -R developer:developer /home/developer

USER developer
WORKDIR /home/developer

CMD ["/bin/bash"]
