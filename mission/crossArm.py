#!/usr/bin/env python3
# coding=utf-8
"""
Author       : zhangjunjie 1157860961@qq.com
Date         : 2024-02-29 16:31:39
LastEditTime : 2024-03-19 21:55:33
LastEditors  : Jay jay.zhangjunjie@outlook.com
Description  : 
"""


import math, time
from rbkSim import SimModule
from rbk import BasicModule, MoveStatus



# =======脚本输入参数=======
"""
####BEGIN DEFAULT ARGS####
{
    "LiftHeight": {
        "value": 0,
        "unit": "mm",
        "max_value":"650",
        "min_value":"275",
        "type": "double",
        "tips": "设定交叉臂当前高度"
    },
    "LiftSpeed":{
        "value": 0,
        "unit": "mm/s",
        "type": "double",
        "tips": "设定交叉臂当前速度"
    }

}
####END DEFAULT ARGS####
"""

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
        remoteLogger = RemoteTCPServerLogHandler("192.168.2.81")
        logger.addHandler(remoteLogger)
    except Exception as e:
        logger.error(e)
    logging.addLevelName(60, "SUCCESS")
    

# ---------------------------------------------------------- 远程日志定义 ----------------------------------------------------------



class Module(BasicModule):

    MOTOR_NAME = "lift"         # motorName
    MAX_VEL = 0.1                 # mm/s
    STOP_DI = 2                 # DI
    HYPOTENUSE = 375 / 2 / 1000       # 
    BOTTOM_LENGTH = 0.366             # 最底部长度

    def __init__(self, rbk: SimModule, args):
        super().__init__()
        self.status = MoveStatus.NONE
        self.arginit = True
        self.argHeight = None
        self.argSpeed = None
        self.rbk = None
        self.startingHeight = 0
        self.timeStamp = 0
        self.timeRate = 22              # default time rate

    def run(self, r: SimModule, args: dict):
        self.rbk = r
        self.status = MoveStatus.RUNNING
        pos, speed = self.getMotorHeightSpeed()  
        if self.arginit:
            self.arginit = False
            self.argHeight = args.get("LiftHeight", None)   # target height     unit:mm
            self.argSpeed = args.get("LiftSpeed", None)     # target speed      unit:mm/s
            self.startingHeight, _ = self.getMotorHeightSpeed()
            self.timeStamp = time.time()
            logger.error(f"args|Height:{self.argHeight},Speed:{self.argSpeed}")
            self.argHeight = (self.argHeight-106) / 1000      # 转换成m
            self.argSpeed  = self.argSpeed / 1000       # 转成m/s
            targetPos = self.liftHeight2MotorPos(self.argHeight)
            if targetPos > pos:
                self.flag = True
            else:
                self.flag = False
                
        else:
            timeSt = time.time()
            self.timeRate = timeSt - self.timeStamp
            self.timeStamp = timeSt

              
        logger.debug(f"MotorPos:{pos}m, MotorSpeed:{speed}m/s")

        if self.argHeight and self.argSpeed:

            targetPos = self.liftHeight2MotorPos(self.argHeight)
            if self.flag:
                if pos >= targetPos : 
                    self.publishMotorPos(targetPos, 0)
                    return MoveStatus.FINISHED
            else:
                if pos <= targetPos:
                    self.publishMotorPos(targetPos , 0)
                    return MoveStatus.FINISHED
            targetSpeed = self.speedPlan(pos, self.argSpeed)
            self.publishMotorPos(targetPos, 0.002)
        
        return MoveStatus.RUNNING


    def getMotorHeightSpeed(self):
        motor = self.rbk.jack()
        pos, speed = motor.get("height"), motor.get("speed")
        return round(pos, 5), round(speed, 5)


    def publishMotorPos(self, pos, speed):
        logger.critical(f"publishMotor|timeRate:{self.timeRate},Pos:{round(pos,5)},Speed:{speed}")
        # self.rbk.setMotorSpeed(self.MOTOR_NAME, speed, self.STOP_DI)
        self.rbk.setMotorPosition(self.MOTOR_NAME, pos, 0.02, self.STOP_DI)
        self.rbk.publishSpeed()
    

    def liftHeight2MotorPos(self, height):
        oneSectionHeight = height / 4
        logger.debug(f"height:{height},oneSectionHeight:{oneSectionHeight}")
        pos = round(math.sqrt(self.HYPOTENUSE**2 - oneSectionHeight**2), 3) * 2
        logger.debug(f"liftHeight2MotorPos|Pos:{pos}")
        logger.debug(f"liftHeight2MotorPos|TargetPos:{self.BOTTOM_LENGTH - pos}")
        return  self.BOTTOM_LENGTH - pos


    def motorPos2LiftHeight(self, pos):
        height = round(math.sqrt(self.HYPOTENUSE**2 - (pos / 2) ** 2), 3) * 4
        return height


    def speedPlan(self, currentPos:float, targetSpeed:float) -> float:
        """
        _summary_

        Args:
            currentPos (_type_): unit:mm
            targetSpeed (_type_): unit:mm/s

        Returns:
            float: _description_
        """
        liftHeight = ((targetSpeed / 1000) * self.timeRate)
        nowHeight = self.motorPos2LiftHeight(currentPos)
        targetPos = self.liftHeight2MotorPos(((nowHeight + liftHeight) / 4))
        actualSpeed = ((targetPos - currentPos) / self.timeRate)     # unit:m/s
        return actualSpeed
        


    def checkMotorPos(self, pos):
        pass


    def cancel(self, r: SimModule):
        self.rbk.stopMotor()


    def suspend(self, r: SimModule):
        return super().suspend(r)
