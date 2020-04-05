#!/bin/bash

distro="melodic"

pkgpath="/src/robomagellan/install"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"
SCRIPT_DIR=${SCRIPT_DIR%$pkgpath}

# Create ROS USER
if ! getent passwd ros > /dev/null; then
    sudo adduser --system ros --home /var/lib/ros --group --shell /bin/bash
fi

# Install /etc/ros config
mkdir -p /etc/ros/$distro
echo "Pointing /etc/ros/$distro at $SCRIPT_DIR"
echo -e "#!/usr/bin/env bash\nsource $SCRIPT_DIR/devel/setup.bash" > /etc/ros/$distro/setup.bash
cp etc/ros/distro/robot.launch_ /etc/ros/$distro/robot.launch

# Install upstart scripts
cp etc/init/*.conf /etc/init/
