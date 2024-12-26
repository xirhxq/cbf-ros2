FROM ros:galactic

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    sudo \
    vim \
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

RUN useradd -m -u 1000 -s /bin/bash appuser \
    && echo "appuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER appuser
WORKDIR /home/appuser

CMD ["/bin/bash"]
