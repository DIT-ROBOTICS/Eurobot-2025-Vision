#!/bin/bash
# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'  
GRBOLD='\033[1;32m'
YELLOW='\033[1;33m'
BLUE='\033[1;34m'    
CYAN='\033[1;36m'

NC='\033[0m' 
# Environment variables
ARCH=$(yq e '.ARCH' init.yml)
BASE_IMAGE=$(yq e '.BASE_IMAGE' init.yml)
DDS=$(yq e '.DDS' init.yml)
ROS_DOMAIN_ID=$(yq e '.ROS_DOMAIN_ID' init.yml)
COMPOSE_PROJECT_NAME=$(yq e '.PROJECT_NAME' init.yml)
COMPOSE_PROFILES=$(yq e '.PROFILES' init.yml)
ACTION=$(yq e '.ACTION' init.yml)

map_target() {
  case "$1" in
    camera) echo "realsense" ;;
    yolo) echo "ultralytics" ;;
    train) return ;;
    *) echo "$1" ;; 
  esac
}

BAKE_TARGETS=""
IFS=',' read -ra PROFILES <<< "$COMPOSE_PROFILES"
for profile in "${PROFILES[@]}"; do
  target=$(map_target "$profile")
  BAKE_TARGETS="$BAKE_TARGETS $target"
done

BAKE_TARGETS=$(echo "$BAKE_TARGETS" | xargs)

update_env_var() {
  local var_name=$1
  local var_value=$2
  local env_file=".env"

  if grep -q "^$var_name=" "$env_file"; then
    sed -i "s/^$var_name=.*/$var_name=$var_value/" "$env_file"
  else
    echo "$var_name=$var_value" >> "$env_file"
  fi
}

case "$ARCH" in
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
    echo "Unsupported ARCH value: $ARCH"
    exit 1
    ;;
esac

case "$DDS" in
  "cyclone")
    RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
    ;;
  "fastdds")
    RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
    ;;
  *)
    echo "Unsupported DDS value: $DDS"
    exit 1
    ;;
esac

case "$ACTION" in
  "build")
    if [ -z "$BAKE_TARGETS" ]; then
      echo -e "${RED}No valid bake targets. Skipping build.${NC}"
    else
      echo -e "Building for      ${BLUE}$ARCH${NC}"
      echo -e "Building Targets  ${BLUE}$BAKE_TARGETS${NC}"
      echo -e "Base Image        ${CYAN}$BASE_IMAGE${NC}"
      echo -e "NVIDIA Base Image ${GRBOLD}$NVIDIA_BASE_IMAGE${NC}"
      echo -e "User UID          ${YELLOW}$(id -u)${NC}"
      sleep 1
      cd dockerfiles
      ARCH=$ARCH \
      BASE_IMAGE=$BASE_IMAGE \
      NVIDIA_BASE_IMAGE=$NVIDIA_BASE_IMAGE \
      USER_UID=$(id -u) \
      docker buildx bake $BAKE_TARGETS
      if [ $? -ne 0 ]; then
        echo "Build failed. Exiting."
        exit 1
      fi
    fi
    ;;
  "run")
    echo -e "Running on    ${BLUE}$COMPOSE_PROJECT_NAME${NC}"
    echo -e "Profiles      ${YELLOW}$COMPOSE_PROFILES${NC}"
    echo -e "ROS DDS       ${CYAN}$RMW_IMPLEMENTATION${NC}"
    echo -e "ROS Domain ID ${GRBOLD}$ROS_DOMAIN_ID${NC}"
    sleep 1
    cd composes
    update_env_var "COMPOSE_PROJECT_NAME" "$COMPOSE_PROJECT_NAME"
    update_env_var "COMPOSE_PROFILES" "$COMPOSE_PROFILES"
    update_env_var "RMW_IMPLEMENTATION" "$RMW_IMPLEMENTATION"
    update_env_var "ROS_DOMAIN_ID" "$ROS_DOMAIN_ID"
    docker compose up -d
    ;;
  "both")
    if [ -z "$BAKE_TARGETS" ]; then
      echo -e "${RED}No valid bake targets. Skipping build.${NC}"
      sleep 1
      echo -e "Running on    ${BLUE}$COMPOSE_PROJECT_NAME${NC}"
      echo -e "Profiles      ${YELLOW}$COMPOSE_PROFILES${NC}"
      echo -e "ROS DDS       ${CYAN}$RMW_IMPLEMENTATION${NC}"
      echo -e "ROS Domain ID ${GRBOLD}$ROS_DOMAIN_ID${NC}"
      sleep 1
      cd composes
      update_env_var "COMPOSE_PROJECT_NAME" "$COMPOSE_PROJECT_NAME"
      update_env_var "COMPOSE_PROFILES" "$COMPOSE_PROFILES"
      update_env_var "RMW_IMPLEMENTATION" "$RMW_IMPLEMENTATION"
      update_env_var "ROS_DOMAIN_ID" "$ROS_DOMAIN_ID"
      docker compose up -d
    else 
      echo -e "Building for      ${BLUE}$ARCH${NC}"
      echo -e "Building Targets  ${BLUE}$BAKE_TARGETS${NC}"
      echo -e "Base Image        ${CYAN}$BASE_IMAGE${NC}"
      echo -e "NVIDIA Base Image ${GRBOLD}$NVIDIA_BASE_IMAGE${NC}"
      echo -e "User UID          ${YELLOW}$(id -u)${NC}"
      sleep 0.5
      cd dockerfiles
      ARCH=$ARCH \
      BASE_IMAGE=$BASE_IMAGE \
      NVIDIA_BASE_IMAGE=$NVIDIA_BASE_IMAGE \
      USER_UID=$(id -u) \
      docker buildx bake $BAKE_TARGETS
      if [ $? -ne 0 ]; then
        echo "Build failed. Exiting."
        exit 1
      fi
      echo -e "${GREEN}Building completed. Now running the image ...${NC}"
      sleep 1
      echo -e "Running on    ${BLUE}$COMPOSE_PROJECT_NAME${NC}"
      echo -e "Profiles      ${YELLOW}$COMPOSE_PROFILES${NC}"
      echo -e "ROS DDS       ${CYAN}$RMW_IMPLEMENTATION${NC}"
      echo -e "ROS Domain ID ${GRBOLD}$ROS_DOMAIN_ID${NC}"
      sleep 1
      cd ../composes
      update_env_var "COMPOSE_PROJECT_NAME" "$COMPOSE_PROJECT_NAME"
      update_env_var "COMPOSE_PROFILES" "$COMPOSE_PROFILES"
      update_env_var "RMW_IMPLEMENTATION" "$RMW_IMPLEMENTATION"
      update_env_var "ROS_DOMAIN_ID" "$ROS_DOMAIN_ID"
      docker compose up -d
    fi
    ;;
  *)
    echo "Unsupported ACTION value: $ACTION"
    exit 1
    ;;
esac