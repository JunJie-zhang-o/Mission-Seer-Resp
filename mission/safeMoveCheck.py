#!/usr/bin/env python3
# coding=utf-8
'''
Author       : Jay jay.zhangjunjie@outlook.com
Date         : 2024-03-31 16:21:02
LastEditTime : 2024-03-31 16:57:27
LastEditors  : Jay jay.zhangjunjie@outlook.com
Description  : AGV移动前的安全检查, 该代码并未测试
'''
import time

from rbkSim import SimModule
from rbk import MoveStatus, BasicModule

from define import ROBOT_IP, DOutDefine, TaskReadCoils, logger, IS_THRER_ROBOT

from userError import Error53901, Error53902, Error53920, Warning55900, Warning55901
from userModbus import ModbusTCP

# =======脚本输入参数=======
"""
####BEGIN DEFAULT ARGS####
{
    "CheckRobotIsHome": {
        "value": false,
        "type": "bool",
        "tips": "true 检测机器人在原点, false 不检测"
    }
}
####END DEFAULT ARGS####
"""

class Module(BasicModule):

    PUSH_IN_TIMEOUT_ERROR = 10                       # 推回检测报警超时时间
    CHECK_ROBOT_HOME_TIMEOUT_ERROR = 10              # 检测机器人原点信号超时时间


    def __init__(self):
        super().__init__()
        self.initArgs = True
        self.status = MoveStatus.NONE
        self.pushInStartT = 0
        self.checkHomeStartT = 0
        self.checkRobotIsHome = False
        self.masterForRobot = ModbusTCP(ip = ROBOT_IP, timeout=0.01)

    def run(self, rbk: SimModule, args: dict):
        self.status = MoveStatus.RUNNING
        if self.initArgs:
            self.checkRobotIsHome = args.get("CheckRobotIsHome", False)
            self.initArgs = False


        self.disenableStableSys(rbk)

        if self.checkRobotIsHome:
            self.robotIsHomeCheck(rbk)


        return self.status

    def robotIsHomeCheck(self, rbk: SimModule):
        if IS_THRER_ROBOT:
            isHome = self.masterForRobot.read_coils(1, TaskReadCoils.IS_HOME_P, 1)[0]
            self.checkHomeStartT = time.time()
            if isHome:
                self.status = MoveStatus.FINISHED
            else:
                self.status = MoveStatus.RUNNING
                if time.time() - self.checkHomeStartT >= self.CHECK_ROBOT_HOME_TIMEOUT_ERROR:
                    self.status = MoveStatus.FAILED
                    rbk.setUserError(Error53920.code, Error53920.msg)


    def disenableStableSys(self, rbk: SimModule):
        # 关闭支撑
        logger.info("disenableStableSys")
        rbk.setDO(DOutDefine.FIXED_CYLINDER, True)
        rbk.setDO(DOutDefine.HOME_OK_LED, False)
        rbk.setDO(DOutDefine.CYLINDER_PUSH_OUT, False)
        rbk.setDO(DOutDefine.CYLINDER_PUSH_IN, True)
        if self.pushInStartT == 0: self.pushInStartT = time.time()
        [manualOrAuto, isForntOk, isBackOk, switch, airWorking] = self.getDI(rbk)
        if (isForntOk & isBackOk) == True:
            self.status = MoveStatus.FINISHED
            logger.critical("disenableStableSys Success")
            rbk.setDO(DOutDefine.FIXED_CYLINDER, False)
            rbk.setDO(DOutDefine.HOME_OK_LED, True)
        else:
            if time.time() - self.pushInStartT >= self.PUSH_IN_TIMEOUT_ERROR:
                rbk.setUserError(Error53902.code, Error53902.msg)
                logger.error(Error53902.msg)
                self.status = MoveStatus.FAILED
                rbk.setDO(DOutDefine.FIXED_CYLINDER, False)
