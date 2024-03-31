# -*- coding: utf-8 -*-
# @Date: 2023/03/20
# @Author: zhong
# @File: modbus_comm.py
# @Version: 1.1
# @Project:
# @Coding:
# @Update: 增加读取预期值

import json
import time
import serial
import sys
from enum import IntEnum
from define import DInDefine, DOutDefine, logger

from userError import Error53901, Error53902, Warning55900, Warning55901

sys.path.append("../modbus_tk")
from rbkSim import SimModule
from rbk import MoveStatus, BasicModule

try:
    import modbus_tk.defines as cst
    from modbus_tk import modbus_tcp, modbus_rtu
except ImportError:
    import os
    os.system("pip install modbus_tk")
    SimModule.setError(SimModule(), f"modbus_tk needs to be installed")
    os.system("pip install modbus_tk -i https://pypi.tuna.tsinghua.edu.cn/simple")
    import modbus_tk.defines as cst
    from modbus_tk import modbus_tcp, modbus_rtu


# =======脚本输入参数=======
"""
####BEGIN DEFAULT ARGS####
{
    "AGVStableSys": {
        "value": false,
        "type": "bool",
        "tips": "ON 支撑, OFF 取消支撑"
    }
}
####END DEFAULT ARGS####
"""


    

class Module(BasicModule):
    """
        车身稳定系统,当AGV移动完成后,机械臂需要动作时,支撑气缸伸出进行支撑,机械臂动作完成后,支撑气缸收回

            气缸移动前,需要先打开锁紧气缸,才可以进行升降

            气缸已经安装在AGV上时,最下方的气管为解锁,最上方的气管为AGV顶升,中间的气管为AGV下降
    """
    AIR_PUMP_TIMEOUT_ERROR = 15                      # 气泵检测报警超时时间
    PUSH_OUT_TIMEOUT_ERROR = 10                       # 推出检测报警超时时间
    PUSH_IN_TIMEOUT_ERROR = 10                       # 推回检测报警超时时间

    
    def __init__(self, rbk: SimModule, args):
        super(Module, self).__init__()
        self.timeout = 60
        self.initArgs = True
        self.status = MoveStatus.NONE
        self.repInfo = dict()
        #ret = os.popen("ls /dev/tty*").read()
        self.isManualTriger = False
        self.airWorkingStartT = 0
        self.pushOutStartT = 0
        self.pushInStartT = 0
        self.manualTrigerFlag = False
        logger.error("init")


    def getDI(self, rbk: SimModule):
        di = rbk.Di()
        manualOrAuto = di["node"][DInDefine.MANUAL_OR_AUTO]["status"]
        isForntOk = di["node"][DInDefine.FRONT_CYLINDER_HOME]["status"]
        isBackOk = di["node"][DInDefine.BACK_CYLINDER_HOME]["status"]
        switch = di["node"][DInDefine.CYLINDER_SWITCH]["status"]
        airWorking = di["node"][DInDefine.AIR_PUMP_IS_WORKING]["status"]
        logger.debug(f"manualOrAuto:{manualOrAuto},isForntOk:{isForntOk},isbackOk:{isBackOk},switch:{switch},airWorking:{airWorking}")
        return [manualOrAuto, isForntOk, isBackOk, switch, airWorking]
        
    
    def getDO(self, rbk:SimModule):
        do = rbk.Do()
        pushIn = do["node"][DOutDefine.CYLINDER_PUSH_IN]["status"]
        pushOut = do["node"][DOutDefine.CYLINDER_PUSH_OUT]["status"]
        fixedCylinder = do["node"][DOutDefine.FIXED_CYLINDER]["status"]
        homeOkLed = do["node"][DOutDefine.HOME_OK_LED]["status"]
        logger.debug(f"pushIn:{pushIn},pushOut:{pushOut},fixedCylinder:{fixedCylinder},homeOkLed:{homeOkLed}")
        return [pushIn, pushOut, fixedCylinder, homeOkLed]
        

    def run(self, rbk: SimModule, args):
        logger.error("run")
        di = self.getDI(rbk)
        do = self.getDO(rbk)
        [manualOrAuto, isForntOk, isBackOk, switch, airWorking] = di
        [pushIn, pushOut, fixedCylinder, homeOkLed]             = do
        self.status = MoveStatus.RUNNING
        # 
        if self.initArgs:
            self.initArgs = False
            # =====参数初始化和参数检查=====
            self.agvStableSys = args.get("AGVStableSys", None)
            self.isManualTriger = args.get("IsManualTrigger", False)

        logger.warning(f"Args self.agvStableSys:{self.agvStableSys} self.isManualTriger:{self.isManualTriger}")
        # =====处理业务逻辑=====
              

        # == 车身稳定系统 == 
              
        if manualOrAuto == False:   # 自动模式
            # 手动模式都响应
            # 自动模式不响应按钮切换
            if self.isManualTriger: 
                logger.warning(Warning55900.msg)
                rbk.setUserWarning(Warning55900.code, Warning55900.msg)
                return MoveStatus.FINISHED

        
        if ((not self.manualTrigerFlag) & self.isManualTriger) == True:
            if pushOut:
                self.agvStableSys = False
                logger.warning(f"manualSwitchAgvStableStatus:{self.agvStableSys}")
            else:
                self.agvStableSys = True
                logger.warning(f"manualSwitchAgvStableStatus:{self.agvStableSys}")
            self.manualTrigerFlag = True
        
       
        
        if self.agvStableSys is None:
            logger.warning(Warning55901.msg)
            rbk.setUserWarning(Warning55901.code, Warning55901.msg)
            return MoveStatus.FINISHED
        else:   
            if self.agvStableSys:
                # 启动支撑
                self.enableStableSys(rbk)
            else:
                # 关闭支撑
                self.disenableStableSys(rbk)
        # == 车身稳定系统 == 

        # =====处理业务逻辑=====


        # =====数据上报及日志打印=====
        self.repInfo['args'] = args
        self.repInfo['AGVStableSys'] = self.agvStableSys
        self.repInfo["isManualTriger"] = self.isManualTriger
        rbk.setInfo(json.dumps(self.repInfo))
        rbk.logInfo(json.dumps(self.repInfo))
        logger.warning(f"Return: {MoveStatus(self.status).name}")
        return self.status


    def enableStableSys(self, rbk: SimModule):
        # 启动支撑
        logger.info("enableStableSys")
        rbk.setDO(DOutDefine.FIXED_CYLINDER, True)
        rbk.setDO(DOutDefine.HOME_OK_LED, False)
        rbk.setDO(DOutDefine.CYLINDER_PUSH_IN, False)
        rbk.setDO(DOutDefine.CYLINDER_PUSH_OUT, True)
        if self.pushOutStartT == 0: self.pushOutStartT = time.time()
        [manualOrAuto, isForntOk, isBackOk, switch, airWorking] = self.getDI(rbk)
        if (isForntOk & isBackOk) == False:
            rbk.setDO(DOutDefine.FIXED_CYLINDER, False)
            self.status = MoveStatus.FINISHED
            logger.critical("enableStableSys Success")
            self.pushOutStartT = 0
            rbk.setDO(DOutDefine.FIXED_CYLINDER, False)
        else:
            if time.time() - self.pushOutStartT >= self.PUSH_OUT_TIMEOUT_ERROR:
                rbk.setUserError(Error53901.code, Error53901.msg)
                logger.error(Error53901.msg)
                self.status = MoveStatus.FAILED
                rbk.setDO(DOutDefine.FIXED_CYLINDER, False)
        
    
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


    def cancel(self, rbk: SimModule):
        # =====处理任务取消时的业务=====
        logger.error("cancel")
        rbk.setNotice(f"cancel task")
        self.status = MoveStatus.NONE


    def suspend(self, rbk: SimModule):
        logger.error("suspend")
        # =====处理任务暂停时的业务=====
        rbk.setNotice(f"suspend task")
        self.status = MoveStatus.SUSPENDED
        
    
    def periodRun(self, rbk: SimModule):
        logger.error("periodRun")


if __name__ == '__main__':  # 本地运行测试
    task1 = [
        {
            "slave": 1,
            "func_code": 3,
            "st_addr": 1,
            "length": 1,
            "expected_value": [5]
        }
    ]
    # args1 = {"operation": "tasks_list", "st_addr": 0, "length": 10, "write_value": [1] * 10}
    # args1 = {"slave_id": 2, "operation": "write_registers", "st_addr": 0, "length": 10, "write_value": [6]*10}
    args1 = {"operation": "read_registers", "st_addr": 0, "length": 2}

    r1 = SimModule()
    m = Module(r1, args1)
    m.run(r1, args1)