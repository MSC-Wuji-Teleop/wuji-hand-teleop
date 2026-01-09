FROM ros:humble

# Change bash as default shell instead of sh
SHELL ["/bin/bash", "-c"]

# 设置非交互模式，避免安装时出现提示
ARG DEBIAN_FRONTEND=noninteractive

# 设置时区
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install essential tools and libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim wget curl git sudo \
    usbutils \
    cmake make \
    python3-pip python3-venv python3-dev \
    python3-numpy python3-yaml \
    && pip3 install --upgrade pip \
    && apt-get autoremove -y && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/*

# For CN user: use command below to replace apt source with tsinghua mirror
# > sudo cp /etc/apt/sources.list.d/ubuntu.sources.cn.bak /etc/apt/sources.list.d/ubuntu.sources
COPY <<EOF /etc/apt/sources.list.d/ubuntu.sources.cn.bak
Types: deb
URIs: http://mirrors.tuna.tsinghua.edu.cn/ubuntu/
Suites: jammy jammy-updates jammy-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF

# 创建工作目录
WORKDIR /workspace

# 设置 ROS2 环境变量
ENV ROS_DISTRO=humble

# 配置 bashrc 自动 source ROS2 环境
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 默认执行命令（自动source ROS2环境）
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && bash"]