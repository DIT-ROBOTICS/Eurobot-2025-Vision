#!/bin/sh
set -e

USERNAME=$1
USER_UID=$2
USER_GID=$3

addgroup --gid "$USER_GID" "$USERNAME"
adduser --uid "$USER_UID" --gid "$USER_GID" --disabled-password --gecos "" "$USERNAME"

mkdir -p /etc/sudoers.d
echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/$USERNAME"
chmod 0440 "/etc/sudoers.d/$USERNAME"

mkdir -p "/home/$USERNAME"
chown "$USERNAME:$USERNAME" "/home/$USERNAME"

echo "source /opt/ros/humble/setup.bash" >> "/home/$USERNAME/.bashrc"
