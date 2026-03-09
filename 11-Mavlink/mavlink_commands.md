

# 设置消息频率

需要指定：

- 实例：通过UDP端口号或者串口设备名，查找对应的mavlink实例；
- 消息名：；
- 消息频率：；

例如：

```shell
mavlink stream -u 14556 -s HEARTBEAT -r 5
```

