# 代码运行说明
运行以下命令启动容器，机器人会依次前往 1、3、5 号矿区。
```
xhost +
docker run -it --network host \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    njtechrobomaster/rmus2022:eval2
```
