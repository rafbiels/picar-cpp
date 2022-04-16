FROM docker.io/arm64v8/ros:galactic-ros-base

# Install basic tools and libraries to support Raspberry Pi 4 interfaces
RUN apt-get update && apt-get install --no-install-recommends -y \
wget curl vim less \
libgpiod-dev i2c-tools \
&& apt-get clean

# Set up repository for GitHub CLI
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null

# Set up repository for LLVM tools
ARG llvm_ver=14
ENV llvm_ver=${llvm_ver:-14}
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add - && \
    echo "deb http://apt.llvm.org/focal/ llvm-toolchain-focal-${llvm_ver} main" >> /etc/apt/sources.list && \
    echo "deb-src http://apt.llvm.org/focal/ llvm-toolchain-focal-${llvm_ver} main" >> /etc/apt/sources.list

# Install GitHub CLI and LLVM tools
RUN apt-get update && apt-get install --no-install-recommends -y \
gh clang-${llvm_ver} lldb-${llvm_ver} clang-tools-${llvm_ver} clang-tidy-${llvm_ver} clangd-${llvm_ver} libc++-${llvm_ver}-dev libc++abi-${llvm_ver}-dev \
&& apt-get clean

# Symlink versioned LLVM tools as unversioned commands
ARG altcmds="clang clang++ clang-tidy clangd lldb"
RUN for cmd in ${altcmds}; do update-alternatives --install /usr/bin/${cmd} ${cmd} /usr/bin/${cmd}-${llvm_ver} 90; done

# Fix default environment to be able to use LLVM tools
COPY llvm.sh /etc/profile.d/llvm.sh
RUN echo '. /etc/profile' >> /opt/ros/galactic/setup.sh

# Create and use a new user dev with sudo rights
RUN echo '%wheel	ALL=(ALL)	NOPASSWD: ALL' >> /etc/sudoers
RUN useradd -m dev && groupadd wheel && adduser dev wheel && usermod --shell /bin/bash dev
USER dev
WORKDIR /home/dev

# Initialise ROS repository
RUN rosdep update
