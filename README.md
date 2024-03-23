



### 车身稳定系统

麦轮AGV标配车身稳定系统，当AGV静止，机械臂需要移动的情况下，车身稳定系统保证整体的稳定性。

车身稳定系统由4个支撑气缸进行实现，当机械臂需要工作时，支撑气缸伸出，对AGV车身进行支撑。

#### 支撑气缸

每个支撑气缸上，分为锁紧和支撑两个气缸，当支撑气缸需要上下移动时，需要先打开/解锁锁紧气缸，再移动支撑气缸，移动完成后，再关闭/上锁 锁紧气缸。

当支撑气缸未伸出时，4个支撑气缸共有4个检测开关，其中未伸出时检测开关有输入，前侧两个检测开关输入到Seer的DI5，后侧两个检测开关共同输入到Seer的DI6。

> 前侧两个检测开关和后侧两个检测开关输入逻辑为：前/后侧，当左侧气缸检测和右侧气缸检测都检测到信号时，Seer的DI有输入。
>
> 当气缸已经安装在AGV上时,最下方的气管为解锁,最上方的气管为AGV顶升,中间的气管为AGV下降
>
> 当支撑气缸检测开关都在原点时（即允许AGV进行移动时），顶升按钮灯会亮绿灯（DO5）

#### 顶升逻辑

车身稳定系统由脚本进行控制，实际通过调度或任务链进行触发使用，或通过按钮进行手动切换触发

##### 通过任务链触发

***顶起***

```json
{
    "operation": "Script",
    "script_args": {
        "AGVStableSys": true
    },
    "script_name": "mission/cylinder.py"
}
```

***收回***

```json
{
    "operation": "Script",
    "script_args": {
        "AGVStableSys": false
    },
    "script_name": "mission/cylinder.py"
}
```

##### 通过按钮手动触发

Seer麦轮AGV上有手自动切换旋钮，和顶升手动切换按钮（自复位按钮）。

当处于手动状态下，按钮顶升按钮，会手动切换车声稳定系统状态（由支撑变为不支撑，或由不支撑变为支撑）

##### 通过调度触发

等待补充

### 气泵工作逻辑

Seer麦轮AGV会一直检测气泵的输入信号（AGV DI:10），如果输入信号为ON连续持续90s，则会报警

> 报警为53900，内容为气泵持续工作中,请检测是否漏气
>
> 如需修改报警时间，在机构脚本中的periodRun.py中，AIR_PUMP_TIMEOUT_ERROR字段，默认为90s

### 自动重定位

Seer在上电后，脚本开始运行后，会自动进行一次重定位（避免无需重定位的场景下，需手动确认定位）。无论重定位是否成功，都会显示脚本自动重定位结束。

> 可以通过触摸屏上AGV的定位状态进行确认当前是否已经重定位成功

### 与机器人的心跳逻辑

Seer麦轮AGV会一直读取机械臂的M535地址，如果5s内发现M535地址并没有更改的话，则会报警

> 报警为53909，内容为AGV 与 Robot Modbus 心跳超时，请检测网络设置和网络链接

### AGV的减速和阻挡信息

Seer麦轮AGV会不断读取AGV本身的减速和阻挡信号，并将减速信息写入到机械臂的M789，同时将阻挡信息写入机械臂的M787

> 其中减速信号可以用来作为机械臂缩减模式使用
>
> 停止信号可以用来做外部暂停使用

### AGV启动机械臂任务

**AGV程序示例**

在任务链中添加自定义动作，内容如下

> 下述中的Site为站点号，Task为任务号
>
> 即在站点1，执行任务2

```json
{
    "operation": "Script",
    "script_args": {
        "Site": 1,
        "Task": 2,
    },
    "script_name": "mission/eliteECArmTask.py"
}
```

**机械臂程序示例**

```tex
NOP
// SeerAGV 调用机械臂任务
// 等待AGV写入启动信号
WAIT MGH#(309) = 1  
// 任务启动，获取写入的站点和任务数据
MIN B001 MGH#(300)
MIN B001 MGH#(301)
// 模拟执行任务逻辑
TIMER T=3 S
// 写入任务完成状态
MOUT MGD#(310) 1
// 写入交互标志，本次任务结束
MOUT MGD#(319) 1
END
```

涉及到的寄存器地址如下：

| 机器人寄存器地址 |        注释        |             读写             |
| :--------------: | :----------------: | :--------------------------: |
|       300        |      站点数据      |        由AGV写入Robot        |
|       301        |      任务数据      |        由AGV写入Robot        |
|       302        |      预留参数      |        由AGV写入Robot        |
|       303        |      预留参数      |        由AGV写入Robot        |
|       304        |      预留参数      |        由AGV写入Robot        |
|       305        |      预留参数      |        由AGV写入Robot        |
|       306        |      预留参数      |        由AGV写入Robot        |
|       307        |      预留参数      |        由AGV写入Robot        |
|       308        |      预留参数      |        由AGV写入Robot        |
|       309        |  数据已经写入完成  |        由AGV写入Robot        |
|                  |                    |                              |
|       310        | 机械臂任务完成状态 | 由Robot自行写入数据，AGV读取 |
|       311        |      预留参数      | 由Robot自行写入数据，AGV读取 |
|       312        |      预留参数      | 由Robot自行写入数据，AGV读取 |
|       313        |      预留参数      | 由Robot自行写入数据，AGV读取 |
|       314        |      预留参数      | 由Robot自行写入数据，AGV读取 |
|       315        |      预留参数      | 由Robot自行写入数据，AGV读取 |
|       316        |      预留参数      | 由Robot自行写入数据，AGV读取 |
|       317        |      预留参数      | 由Robot自行写入数据，AGV读取 |
|       318        |      预留参数      | 由Robot自行写入数据，AGV读取 |
|       319        |  数据已经写入完成  | 由Robot自行写入数据，AGV读取 |

