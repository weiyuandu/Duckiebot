One-time setup:  
0. Windows OS should install WSL

1. Install docker inside WSL
2. Run：docker pull duckietown/gym-duckietown
3. Create C:\\duckiesim on Windows host
4. Run：docker run -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/c/duckiesim:/gym\_duckietown--env="QT\_X11\_NO\_MITSHM=1" --name duckie duckietown/gym-duckietown bash
5. Run in container：apt update \&\& apt install -y fontconfig libglib2.0-0
6. Run in container：pip install stable\_baselines3
7. Exit container：exit
8. Save changes：docker commit duckie updated\_duckietown\_image

From now on, start the simulator with：docker start -ai duckie  

Run in container: <your script> or test your setup with python manual\_control.py

Exit simulator：Ctrl+C

