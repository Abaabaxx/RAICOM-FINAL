    #!/bin/bash
    
    # 设置地图状态文件的保存目标目录
    MAP_DIR="/home/tianbot/tianbot_ws/src/lby_test/param/carto_map"
    
    # 动态生成文件名
    FILENAME="map_$(date +'%Y_%m_%d_%H_%M').pbstream"
    
    # 确保目录存在
    mkdir -p $MAP_DIR
    
    echo "正在保存 Cartographer 状态..."
    echo "目标服务: /Tianracer/write_state"
    echo "保存路径: $MAP_DIR/$FILENAME"
    
    # 执行保存命令
    rosservice call /Tianracer/write_state "{filename: '$MAP_DIR/$FILENAME'}"
            
    # 检查命令是否成功执行
    if [ $? -eq 0 ]; then
      echo "Cartographer 状态保存成功！"
    else
      echo "错误: Cartographer 状态保存失败。" >&2
    fi