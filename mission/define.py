from enum import IntEnum

# 
ROBOT_IP = "192.168.192.205"             # 机械臂的IP
IS_THRER_ROBOT = True                  # 是否有机械臂


# ---------------------------------------------------------- 远程日志定义 ----------------------------------------------------------
import logging
import logging.handlers

class RemoteTCPServerLogHandler(logging.StreamHandler):
    """
        适用于原生logging的远端TCP服务器日志记录
    """
    def __init__(self, host, port=logging.handlers.DEFAULT_TCP_LOGGING_PORT):
        logging.StreamHandler.__init__(self)
        import socket
        self.client = socket.socket(socket.AF_INET,socket.SOCK_STREAM) 
        self.tcpServerAddr = (host, port) 
        self.client.connect(self.tcpServerAddr)  
        self.stream = self.client.makefile("wr")
        selfIP = self.client.getsockname()[0]
        selfPort = self.client.getsockname()[1]
        self.formatter = logging.Formatter("%(asctime)s | " + f"Client IP:{selfIP}:{selfPort}| " +"%(levelname)-8s |%(filename)s:%(lineno)-4d | %(message)s")


# 添加remoteLogger
logger = logging.getLogger('Mission Seer')
logger.setLevel(logging.DEBUG)
if not logger.hasHandlers():
    try:
        remoteLogger = RemoteTCPServerLogHandler("192.168.2.11")
        logger.addHandler(remoteLogger)
    except Exception as e:
        logger.error(e)
    logging.addLevelName(60, "SUCCESS")
    

# ---------------------------------------------------------- 远程日志定义 ----------------------------------------------------------




# -------------------------------------------------- IO Define --------------------------------------------------
class DInDefine(IntEnum):
    """输入定义
    """
    MANUAL_OR_AUTO        = 0       # 0:自动  1:手动
    FRONT_CYLINDER_HOME   = 5       # 前侧两个气缸原点,推回时有输入
    BACK_CYLINDER_HOME    = 6       # 后侧两个气缸原点,推回时有输入
    CYLINDER_SWITCH       = 9       # 手动支撑气缸切换按钮, 0为不控制,1为仅在手动模式下控制
    AIR_PUMP_IS_WORKING   = 10      # 气泵正在工作中

class DOutDefine(IntEnum):
    """输出定义
    """
    AIR_TIMEOUT_OUT     = 1         # 气泵长时间工作后,需要超时报警
    CYLINDER_PUSH_OUT   = 2         # 气缸推出
    CYLINDER_PUSH_IN    = 3         # 气缸退回
    FIXED_CYLINDER      = 4         # 固定气缸,当气缸推出后,进行固定 #! 当气缸需要移动前,需要先打开该气缸,移动完成后,关闭该气缸进行锁紧
    HOME_OK_LED         = 5         # 顶升到位信号

# -------------------------------------------------- IO Define --------------------------------------------------


# --------------------------------------- AGV < -- > Robot Register Define --------------------------------------
class TaskWriteRegister(IntEnum)    : 
    # AGV -> Robot
    RESERVE_0                       = 300
    RESERVE_1                       = 301
    RESERVE_2                       = 302
    RESERVE_3                       = 303
    SITE_NUM                        = 304               # 站点数据
    TASK_NUM                        = 305               # 任务数据
    RESERVE_6                       = 306
    RESERVE_7                       = 307
    RESERVE_8                       = 308
    DATA_OK_SIGN                    = 309               # 数据已经写入完成
    
class TaskWriteCoils(IntEnum)        : 
    INIT                            = 784               # 初始化
    START                           = 785               # 使机器人启动
    STOP                            = 786               # 使机器人停止
    PAUSE                           = 787               # 使机器人暂停
    RESUME                          = 788               # 使机器人复位
    SLOW_DOWN                       = 789               # 使机器人减速
    

class TaskReadRegister(IntEnum)     : 
    # Robot -> AGV
    TASK_STATE                      = 310               # 任务完成状态
    RESERVE_1                       = 311
    CLEAR_FLAG                      = 312               # 数据已经写入完成
    RESERVE_3                       = 313
    RESERVE_4                       = 314
    RESERVE_5                       = 315
    RESERVE_6                       = 316
    RESERVE_7                       = 317
    RESERVE_8                       = 318
    RESERVE_9                       = 319             
    
    
class TaskReadCoils(IntEnum)         : 
    IS_HOME_P                       = 528               # Robot在原点
    IS_SAFE_1_REGION                = 529               # Robot在安全区域1
    IS_SAFE_2_REGION                = 530               # Robot在安全区域2
    ROBOT_IS_ALARM                  = 531               # Robot为报警状态
    ROBOT_IS_PAUSE                  = 532               # Robot为暂停状态
    ROBOT_IS_RUNNING                = 533               # Robot为运行状态
    ROBOT_IS_STOP                   = 534               # Robot为停止状态
    ROBOT_HEARTBEAT                 = 535               # Robot心跳 

# --------------------------------------- AGV < -- > Robot Register Define --------------------------------------


class AGVReadCoils(IntEnum):
    IS_SLOW_DOWN    = 0     # AGV是否减速
    IS_BLOCK        = 1     # AGV是否阻挡
    IS_CHARGING     = 2     # AGV是否充电中
    IS_ESTOP        = 3     # AGV是否急停
    