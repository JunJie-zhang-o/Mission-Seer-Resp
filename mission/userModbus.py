
# todo:使用该脚本请先上传modbus_tk package
import serial
import modbus_tk.defines as cst
from modbus_tk import modbus_tcp, modbus_rtu



class ModbusTCP:
    """
    线圈 - 可读可写布尔量
    离散输入 - 只读布尔量
    保持寄存器 - 可读可写寄存器(16位)
    输入寄存器 - 只读寄存器(16位)

    # supported modbus functions
    READ_COILS = 1
    READ_DISCRETE_INPUTS = 2
    READ_HOLDING_REGISTERS = 3
    READ_INPUT_REGISTERS = 4
    WRITE_SINGLE_COIL = 5
    WRITE_SINGLE_REGISTER = 6
    WRITE_MULTIPLE_COILS = 15
    WRITE_MULTIPLE_REGISTERS = 16
    """

    def __init__(self, ip='127.0.0.1', port=502, timeout=3.0):
        self.master = modbus_tcp.TcpMaster(ip, port, timeout)

    def read_coils(self, slave, st_addr=0, length=1) -> tuple:
        """
        读取线圈
        @param slave:从机ID
        @param st_addr:起始地址
        @param length:读取长度
        @return:
        """
        return self.master.execute(slave, cst.READ_COILS, st_addr, length)

    def read_discrete_inputs(self, slave, st_addr=0, length=1) -> tuple:
        """
        读取离散输入
        @param slave:从机ID
        @param st_addr:起始地址
        @param length:读取长度
        @return:
        """
        return self.master.execute(slave, cst.READ_DISCRETE_INPUTS, st_addr, length)

    def read_holding_registers(self, slave, st_addr=0, length=1) -> tuple:
        """
        读取保持寄存器
        @param slave:从机ID
        @param st_addr:起始地址
        @param length:读取长度
        @return:
        """
        return self.master.execute(slave, cst.READ_HOLDING_REGISTERS, st_addr, length)

    def read_input_registers(self, slave, st_addr=0, length=1) -> tuple:
        """
        读取输入寄存器
        @param slave:从机ID
        @param st_addr:起始地址
        @param length:读取长度
        @return:
        """
        return self.master.execute(slave, cst.READ_INPUT_REGISTERS, st_addr, length)

    def write_single_coil(self, slave, st_addr, output_value):
        """
        写入单线圈
        @param slave:从机ID
        @param st_addr:起始地址
        @param output_value:待写入的数据
        @return:
        """
        return self.master.execute(slave, cst.WRITE_SINGLE_COIL, st_addr, output_value=output_value)

    def write_multi_coils(self, slave, st_addr, output_value):
        """
        写入多线圈
        @param slave:从机ID
        @param st_addr:起始地址
        @param output_value:待写入的数据
        @return:
        """
        return self.master.execute(slave, cst.WRITE_MULTIPLE_COILS, st_addr, output_value=output_value)

    def write_single_register(self, slave, st_addr, output_value):
        """
        写单一寄存器
        @param slave:从机ID
        @param st_addr:起始地址
        @param output_value:待写入的数据
        @return:
        """
        return self.master.execute(slave, cst.WRITE_SINGLE_REGISTER, st_addr, output_value=output_value)

    def write_multi_registers(self, slave, st_addr, output_value: list):
        """
        写多寄存器
        @param slave:从机ID
        @param st_addr:起始地址
        @param output_value:待写入的数据
        @return:
        """
        return self.master.execute(slave, cst.WRITE_MULTIPLE_REGISTERS, st_addr, output_value=output_value)


class ModbusRTU(ModbusTCP):
    def __init__(self, port, baudrate=9600, bytesize=8, parity='N', stopbits=1, xonxoff=0, timeout=1.0):
        """
        Modbus-RTU协议串口通信
        @param port: 串口
        @param baudrate: 波特率
        @param bytesize: 字节大小
        @param parity: 校验位
        @param stopbits: 停止位
        @param xonxoff: 读超时
        @param timeout: 写超时
        """
        self.master = modbus_rtu.RtuMaster(serial.Serial(port=port, baudrate=baudrate, bytesize=bytesize,
                                                             parity=parity, stopbits=stopbits, xonxoff=xonxoff))
        self.master.set_timeout(timeout)


if __name__ == "__main__":
    t1 = ModbusTCP()
    t1.write_multi_registers(1, 300, [1])
    t1.write_single_register(1, 300, 12)