# DUA environment image.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# April 5, 2023

FROM intelligentsystemslabutv/dua-foundation:x86-cudev

# Change this if you encounter problems with the default user
ARG USER_UID=1000

ENV DEBIAN_FRONTEND=noninteractive

# IMAGE SETUP START #
# ldc23-simulator START #
# Install PX4 general dependencies (should be required by the plugin)
RUN apt-get update && apt-get install -y --no-install-recommends \
  astyle \
  libxml2-dev \
  libgstreamer-plugins-base1.0-dev \
  rsync \
  python3-setuptools \
  python3-wheel && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

# Install PX4 Python general dependencies (should be required by the plugin)
RUN yes | pip3 install -U \
  future \
  jinja2>=2.8 \
  jsonschema \
  kconfiglib \
  lxml \
  psutil \
  pygments \
  pymavlink \
  pyulog

# Build and install MAVLink
# We need our fork because PX4 requires the development dialect, which somehow does not build
# by default
WORKDIR /opt
RUN git clone --single-branch --branch 'px4-v1.13.3' --recursive https://github.com/IntelligentSystemsLabUTV/mavlink.git && \
  cd mavlink && \
  python3 -m pymavlink.tools.mavgen \
    --lang=C \
    --wire-protocol=2.0 \
    --output=generated/include/mavlink/v2.0 \
    message_definitions/v1.0/development.xml && \
  cmake -Bbuild -H. -DCMAKE_INSTALL_PREFIX=install && \
  cmake --build build --target install && \
  cd .. && \
  chgrp -R internal mavlink && \
  chmod -R g+rw mavlink
WORKDIR /root
# ldc23-simulator END #
# IMAGE SETUP END #

# Create a user with Zsh as shell, hashed password, and add it to the sudoers
# To generate the hashed password, run:
#   mkpasswd -m sha-512 PASSWORD intelsyslab
RUN useradd -r -m -s /usr/bin/zsh -u ${USER_UID} -G adm,dialout,internal,plugdev,sudo,tty,video -p '$6$intelsyslab$DVn8eT24upAB4SSgZMpYiv29SewU3ueUjBMaj3ublhs0NKQCItbKNmIOS/Skie.vQbqlfOvlo2Yoeu4OeUypk0' neo
ENV HOME=/home/neo

# Create workspace directory: host workspaces will be mounted here
RUN mkdir ${HOME}/workspace && \
    chown neo:neo ${HOME}/workspace

# Create directory for shell history file
RUN mkdir ${HOME}/zsh_history && \
    chown neo:neo ${HOME}/zsh_history

# Create SSH directory for user
RUN mkdir ${HOME}/.ssh

# Switch to internal user
USER neo
WORKDIR ${HOME}

# Copy user configuration files
COPY --chown=neo:neo ./aliases.sh ./.aliases.sh
COPY --chown=neo:neo ./bashrc ./.bashrc
COPY --chown=neo:neo ./colcon-defaults.yaml /home/neo/.colcon/defaults.yaml
COPY --chown=neo:neo ./commands.sh ./.commands.sh
COPY --chown=neo:neo ./nanorc ./.nanorc
COPY --chown=neo:neo ./ros2.sh ./.ros2.sh
COPY --chown=neo:neo ./vimrc ./.vimrc

# Configure Zsh for internal user
ENV ZSH=${HOME}/.oh-my-zsh
ENV ZSH_CUSTOM=${ZSH}/custom
ENV ZSH_PLUGINS=${ZSH_CUSTOM}/plugins
ENV ZSH_THEMES=${ZSH_CUSTOM}/themes
RUN wget -qO- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh | zsh || true
RUN git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_PLUGINS}/zsh-syntax-highlighting \
    && git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-autosuggestions ${ZSH_PLUGINS}/zsh-autosuggestions \
    && git clone --single-branch --depth 1 https://github.com/romkatv/powerlevel10k.git ${ZSH_THEMES}/powerlevel10k
COPY --chown=neo:neo ./p10k.zsh ./.p10k.zsh
COPY --chown=neo:neo ./zshrc ./.zshrc

ENV DEBIAN_FRONTEND=dialog

# Add image information
LABEL author.name="Roberto Masocco"
LABEL author.email="robmasocco@gmail.com"
LABEL organization.name="Intelligent Systems Lab"
LABEL organization.email="isl.torvergata@gmail.com"

# By default, start a basic shell
CMD ["bash"]
