# 代码运行说明
运行以下命令启动容器，比赛程序会自动开始运行。
```
xhost +
docker run -it --network host \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    njtechrobomaster/rmus2022:final
```
