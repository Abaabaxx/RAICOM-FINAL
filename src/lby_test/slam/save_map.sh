#!/bin/bash

# 设置地图保存的目标目录
MAP_DIR="/home/tianbot/tianbot_ws/src/lby_test/config/map"

# 动态生成文件名
FILENAME="map_$(date +'%Y_%m_%d_%H_%M')"

# 确保目录存在
mkdir -p $MAP_DIR

echo "正在保存地图..."
echo "目标话题: /Tianracer/map"
echo "保存路径: $MAP_DIR/$FILENAME"

# 执行保存命令
rosrun map_server map_saver -f "$MAP_DIR/$FILENAME" map:=/Tianracer/map

echo "地图保存成功！"