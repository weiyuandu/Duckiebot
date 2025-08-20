只需运行一次:  
0. Windows系统，安装wsl（详见官网）
1. 安装 docker （详见官网）
2. 运行命令：docker pull duckietown/gym-duckietown
3. 在C盘根下建立文件夹duckiesim
4. 运行命令：docker run -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/c/duckiesim:/workspace --env="QT_X11_NO_MITSHM=1" --name duckie duckietown/gym-duckietown bash
5. 容器内运行命令：apt update && apt install -y fontconfig libglib2.0-0
7. 退出容器：exit
8. 保存更改：docker commit duckie updated_duckietown_image

之后打开模拟器容器可以直接运行：
docker start -ai duckie  
./manual_control.py --env-name Duckietown-udem1-v0  
容器内运行命令：./manual_control.py --env-name Duckietown-udem1-v0  
退出模拟器：Ctrl+C 或 关闭模拟器窗口
  
如果一切正常，应该能看到模拟器gui窗口，可以用上下左右控制
