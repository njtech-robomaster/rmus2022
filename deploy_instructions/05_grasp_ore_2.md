# 代码运行说明
运行以下命令启动容器，机器人会抓取其前方的 **1 号**矿石。
```
xhost +
docker run -it --network host \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    njtechrobomaster/rmus2022:evalgrasp
```
