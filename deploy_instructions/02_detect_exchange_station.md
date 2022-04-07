# 代码运行说明
运行以下命令启动容器，识别结果会在 `Marker Detections` 窗口中显示。
```
docker run -it --network host \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    njtechrobomaster/rmus2022:eval1
```
