




    
from rbk import BasicModule
from rbkSim import SimModule
import time
from define import AGVReadCoils, DInDefine, DOutDefine, TaskReadCoils, TaskWriteCoils, ROBOT_IP, IS_THRER_ROBOT, logger
from userError import Error53900, Error53909
from userModbus import ModbusTCP



class BasePeriodRun():
    
    @classmethod
    def run(cls, rbk:SimModule):
        pass


class AgvToRobot(BasePeriodRun):
    """
        AGV 的减速和阻挡信号等给到机器人
    """
    masterForAgv = ModbusTCP(ip = "192.168.192.5", timeout=0.01)
    masterForRobot = ModbusTCP(ip = ROBOT_IP, timeout=0.01)
    DEFAULT_SLAVE_ID = 1
    lastSlowDownSign, lastBlockSign = False, False
    
    @classmethod
    def run(cls, rbk:SimModule):
        logger.info("AgvToRobot")
        isSlowDown = cls.masterForAgv.read_discrete_inputs(cls.DEFAULT_SLAVE_ID, AGVReadCoils.IS_SLOW_DOWN, 1)[0]
        isBlock = cls.masterForAgv.read_discrete_inputs(cls.DEFAULT_SLAVE_ID, AGVReadCoils.IS_BLOCK, 1)[0]
        # logger.debug(str(isSlowDown)+str(isBlock)+str(cls.lastSlowDownSign)+str(cls.lastBlockSign))
        if IS_THRER_ROBOT:
            if cls.lastSlowDownSign != isSlowDown:
                cls.masterForRobot.write_single_coil(cls.DEFAULT_SLAVE_ID, TaskWriteCoils.SLOW_DOWN, isSlowDown)
                cls.lastSlowDownSign = isSlowDown
            if cls.lastBlockSign != isBlock:
                cls.masterForRobot.write_single_coil(cls.DEFAULT_SLAVE_ID, TaskWriteCoils.PAUSE, isBlock)
                cls.lastBlockSign = isBlock



class HeartBeatAgvAndRobot(BasePeriodRun):
    """
        AGV 与 机器人的心跳处理
    """

    beatTime = 5    # 心跳节拍
    masterForRobot = ModbusTCP(ip = ROBOT_IP)
    DEFAULT_SLAVE_ID = 1
    heatBeat = None
    lastHeatBeatTime = 0
    
    @classmethod
    def run(cls, rbk:SimModule):
        logger.info("HeartBeat")
        if IS_THRER_ROBOT:
            state = cls.masterForRobot.read_coils(cls.DEFAULT_SLAVE_ID, TaskReadCoils.ROBOT_HEARTBEAT, 1)[0]
            if cls.heatBeat is None: 
                cls.heatBeat == state
                cls.lastHeatBeatTime = time.time()
            if cls.heatBeat == state:
                if time.time() - cls.lastHeatBeatTime >= cls.beatTime:
                    rbk.setUserError(Error53909.code, Error53909.msg)
                    logger.error(Error53909.msg)
            else:
                cls.lastHeatBeatTime = time.time()
                cls.heatBeat = state
                if rbk.errorExits(Error53909.code): rbk.clearError(Error53909.code)



class AirPumpTimeOutCheck(BasePeriodRun):
    """
        监测气泵的输入信号,如果气泵持续有输入信号,则报警并输出切断气泵供电
    """
    AIR_PUMP_TIMEOUT_ERROR = 90                      # 气泵检测报警超时时间
    airWorkingStartT = 0

    @classmethod
    def run(cls, rbk:SimModule):
        logger.info("AirPump")
        airWorking = rbk.Di()["node"][DInDefine.AIR_PUMP_IS_WORKING]["status"]
        airForcePowerOff = rbk.Do()["node"][DOutDefine.AIR_TIMEOUT_OUT]["status"]
        airTimeoutDout = False
        # == 气泵判断 == 
        if airWorking == True and cls.airWorkingStartT == 0:
            cls.airWorkingStartT = time.time()
            if rbk.errorExits(Error53900.code): rbk.clearError(Error53900.code)
        elif airWorking == True:
            t = time.time()
            if t - cls.airWorkingStartT >= cls.AIR_PUMP_TIMEOUT_ERROR:
                rbk.setUserError(Error53900.code, Error53900.msg)
                logger.error(Error53900.msg)
                airTimeoutDout = True
                cls.airWorkingStartT = t                # 每个超时周期内触发一次,防止触发报警清除
        else:
            cls.airWorkingStartT = 0
            if rbk.errorExits(Error53900.code): rbk.clearError(Error53900.code)
        if airTimeoutDout:  # 只设置为True
            rbk.setDO(DOutDefine.AIR_TIMEOUT_OUT, airTimeoutDout)

        # == 气泵判断 == 



class Repositioning(BasePeriodRun):
    """
        当小车首次上电时,当需要重定位时,进行一次自动重定位
    """
    needRepositioned = True
    
    AGV_POSTTION_STATE_INPUT_REG = 8 - 1        # AGV定位状态
    
    REPOSITION_COIL = 3 - 1                     # 确定重定位状态
    GET_CONTROLLER_COIL = 10 - 1                # 获取控制权
    RELEASE_CONTROLLER_COIL = 11 - 1            # 释放控制权
    
    agvMaster = ModbusTCP(ip="192.168.192.5", timeout=0.01)
    
    DEFAULT_SLAVE_ID = 1
    
    @classmethod
    def run(cls, rbk: SimModule):
        if cls.needRepositioned:
            logger.info("Repositioning")
            positionState = cls.agvMaster.read_input_registers(cls.DEFAULT_SLAVE_ID, cls.AGV_POSTTION_STATE_INPUT_REG, 1)[0]
            logger.info(positionState)
            if positionState == 3:  # 当定位状为定位完成时,主动获取控制权,写入定位成功,并释放控制权
                cls.agvMaster.write_single_coil(cls.DEFAULT_SLAVE_ID, cls.GET_CONTROLLER_COIL, 1)
                cls.agvMaster.write_single_coil(cls.DEFAULT_SLAVE_ID, cls.REPOSITION_COIL, 1)
                cls.agvMaster.write_single_coil(cls.DEFAULT_SLAVE_ID, cls.RELEASE_CONTROLLER_COIL, 1)
                cls.needRepositioned = False
                logger.info("Repositioning end")
                rbk.setNotice("脚本自动重定位结束")



class CustomErrorWavPlay(BasePeriodRun):
    """
        检查是否存在自定义报错和音频,如果存在则播放对应的错误音频
    """

    @classmethod
    def run(cls, rbk:SimModule):
        pass


class Module(BasicModule):
    """
        每个运行周期运行,MainLoop
    """
    def __init__(self, r: SimModule, args):
        super().__init__()
        
    
    def periodRun(self, rbk:SimModule):
        try:
            AirPumpTimeOutCheck.run(rbk)
            AgvToRobot.run(rbk)
            HeartBeatAgvAndRobot.run(rbk)
            Repositioning.run(rbk)
        except Exception as e:
            logger.exception(e)