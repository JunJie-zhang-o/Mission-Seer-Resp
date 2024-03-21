



from enum import IntEnum
import sys
from define import TaskReadRegister, TaskWriteCoils, TaskWriteRegister, ROBOT_IP, logger
from userError import Error53910

from rbkSim import SimModule


from rbk import BasicModule, MoveStatus
from userModbus import ModbusTCP




# =======脚本输入参数=======
"""
####BEGIN DEFAULT ARGS####
{
    "Site": {
        "value": 0,
        "type": "int",
        "tips": "站点号,机器人判断该标志不为0即可开始动作"
    },
    "Task": {
        "value": 0,
        "type": "int",
        "tips": "用来指示机器人具体执行的是哪一个任务,如只有一个任务,该参数可以忽略"
    }
}
####END DEFAULT ARGS####
"""






class Module(BasicModule):
    
    DEFAULT_SLAVE_ID = 1
    
    
    def __init__(self, rbk: SimModule, args):
        super().__init__()
        self.status = MoveStatus.NONE
        self.argInit = True
        self.masterForRobot = ModbusTCP(ip=ROBOT_IP, port=502)
        self.masterForAGV   = ModbusTCP(ip="192.168.192.5", port=502)
        self.regSite = None
        self.regTask = None


    def run(self, rbk: SimModule, args: dict):
        # 暂停后恢复运行
        if self.status == MoveStatus.SUSPENDED:
            self.masterForRobot.write_single_coil(self.DEFAULT_SLAVE_ID, TaskWriteCoils.RESUME, 1)

        self.status = MoveStatus.RUNNING
        
        if self.argInit:
            self.argInit = False
            self.regSite = args.get("Site", None)
            self.regTask = args.get("Task", None)

            if self.regSite and self.regTask:
                self.masterForRobot.write_single_register(self.DEFAULT_SLAVE_ID, TaskWriteRegister.SITE_NUM, self.regSite)
                self.masterForRobot.write_single_register(self.DEFAULT_SLAVE_ID, TaskWriteRegister.TASK_NUM, self.regTask)
                self.masterForRobot.write_single_register(self.DEFAULT_SLAVE_ID, TaskWriteRegister.WRITE_DATA_OK_SIGN, 1)
        
        # 读取完成信号,处理任务状态
        flag = self.masterForRobot.read_holding_registers(self.DEFAULT_SLAVE_ID, TaskReadRegister.READ_DATA_FLAG, 1)[0]
        if flag:
            taskExecuteState = self.masterForRobot.read_holding_registers(self.DEFAULT_SLAVE_ID, TaskReadRegister.TASK_STATE, 1)[0]
            if taskExecuteState == 1:
                self.status = MoveStatus.FINISHED
            else:
                self.status = MoveStatus.FAILED
                rbk.setUserError(Error53910.code, Error53910.msg)
        
        return self.status
        
    
    def cancel(self, rbk: SimModule):
        # 取消机械臂的任务执行
        self.status = MoveStatus.NONE
        self.masterForRobot.write_single_coil(self.DEFAULT_SLAVE_ID, TaskWriteCoils.STOP, 1)

    
    def suspend(self, rbk: SimModule):
        # 暂停机械臂的任务执行
        self.status = MoveStatus.SUSPENDED
        self.masterForRobot.write_single_coil(self.DEFAULT_SLAVE_ID, TaskWriteCoils.PAUSE, 1)
    
    
    def periodRun(self, rbk: SimModule):
        pass
    
    
    