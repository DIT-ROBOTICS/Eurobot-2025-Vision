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

echo "Supported PLATFORM: $ARCH"
echo "NVIDIA_BASE_IMAGE: $NVIDIA_BASE_IMAGE"

docker compose up -d 
