

class UserError():

    def __init__(self, code:int, msg:str) -> None:
        self.code:int = code
        self.msg:str = msg
    
class UserWarning():

    def __init__(self, code:int, msg:str) -> None:
        self.code:int = code
        self.msg:str = msg    
    
Error53900 = UserError(53900, "气泵持续工作中,请检测是否漏气")


Error53901 = UserError(53901, "气缸推出超时,请检测气压与对应的气缸")
Error53902 = UserError(53902, "气缸推回超时,请检测气压与对应的检测开关")
Error53903 = UserError(53903, "")
Error53904 = UserError(53904, "")
Error53905 = UserError(53905, "")
Error53906 = UserError(53906, "")
Error53907 = UserError(53907, "")
Error53908 = UserError(53908, "")
Error53909 = UserError(53909, "底盘 与 机器人通讯 心跳超时,请检测网络设置和网络链接")




Error53910 = UserError(53910, "机械臂任务执行失败,请在机械臂上查看具体原因")


Error53920 = UserError(53920, "安全移动检查失败,机械臂不在原点,请查看机械臂状态")




Warning55900 = UserWarning(55900, "自动模式下不处理车身稳定系统切换按钮")
Warning55901 = UserWarning(55901, "未输入车身稳定系统参数,不进行稳定系统控制")
