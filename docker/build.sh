#!/bin/bash
WIFI_INTERFACE=$(nmcli device status | awk '$2 == "wifi" {print $1}')
ETH_INTERFACE=$(nmcli device status | awk '$2 == "ethernet" {print $1}')

set -a
source .env
set +a

case "$PLATFORM" in
  "amd64")
    ARCH="linux/amd64"
    NVIDIA_BASE_IMAGE="nvcr.io/nvidia/pytorch:24.07-py3"
    ;;
  "arm64")
    ARCH="linux/arm64"
    NVIDIA_BASE_IMAGE="nvcr.io/nvidia/pytorch:24.07-py3"
    ;;
  "arm64-igpu")
    ARCH="linux/arm64"
    NVIDIA_BASE_IMAGE="nvcr.io/nvidia/pytorch:24.07-py3-igpu"
    ;;
  *)
    echo "Unsupported PLATFORM value: $PLATFORM"
    exit 1
    ;;
esac

sed -i "s|^ARG ARCH.*|ARG ARCH=$ARCH|" Dockerfile
sed -i "s|^ARG NVIDIA_BASE_IMAGE.*|ARG NVIDIA_BASE_IMAGE=$NVIDIA_BASE_IMAGE|" Dockerfile
sed -i "0,/<NetworkInterface name=\"[^\"]*\"/ s/<NetworkInterface name=\"[^\"]*\"/<NetworkInterface name=\"$WIFI_INTERFACE\"/" ../config/dds-uri/cycloneDDS.xml

echo -e "Supported PLATFORM  \033[33m$ARCH\033[0m"
echo -e "NVIDIA_BASE_IMAGE   \033[32m$NVIDIA_BASE_IMAGE\033[0m"
echo -e "NetworkInterface    \033[34m$WIFI_INTERFACE\033[0m"

if [ "$CACHE" == "false" ]; then
  echo -e "Build \033[31m--no-cache\033[0m"
  docker compose build --no-cache --parallel
  docker compose up -d
else
  echo -e "Build \033[32m--cache\033[0m"
  docker compose up -d
fi

