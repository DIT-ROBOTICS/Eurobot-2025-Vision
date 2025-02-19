#!/bin/bash

set -a
source .env
set +a

if [ "$PLATFORM" == "amd64" ]; then
  ARCH="linux/amd64"
  NVIDIA_BASE_IMAGE="nvcr.io/nvidia/pytorch:24.07-py3"

elif [ "$PLATFORM" == "arm64" ]; then
  ARCH="linux/arm64"
  NVIDIA_BASE_IMAGE="nvcr.io/nvidia/pytorch:24.07-py3"

elif [ "$PLATFORM" == "arm64-igpu" ]; then
  ARCH="linux/arm64"
  NVIDIA_BASE_IMAGE="nvcr.io/nvidia/pytorch:24.07-py3-igpu"

else
  echo "Unsupported PLATFORM value: $PLATFORM"
  exit 1
fi

sed -i "s|^ARG ARCH.*|ARG ARCH=$ARCH|" Dockerfile
sed -i "s|^ARG NVIDIA_BASE_IMAGE.*|ARG NVIDIA_BASE_IMAGE=$NVIDIA_BASE_IMAGE|" Dockerfile

echo -e "Supported PLATFORM  \033[33m$ARCH\033[0m"
echo -e "NVIDIA_BASE_IMAGE   \033[32m$NVIDIA_BASE_IMAGE\033[0m"

if [ "$CACHE" == false ]; then
  echo -e "Build \033[31m--no-cache\033[0m"
  docker compose build --no-cache --parallel
  docker compose up -d
else
  echo -e "Build \033[32m--cache\033[0m"
  docker compose up -d
fi

