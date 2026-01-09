"""
模块功能描述：
单通道ascii模式数据采集、解码、分析，然后通过管道传给机械臂c++程序
*********************************
版本：1.0
最近一次修改日期：2024-12-10

修改日志：
2024-10-30，建立初版
2024-12-10：修改了发送数据模式。原来是一次性发送多个数据，现在改为逐条发送。
"""
import os
import logging
from typing import Optional, List, Dict, Any, Generator
import serial
import time
import struct


# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class AsciiSendModel:
    """
    关于传感器ascii模式的类，包括参数设置和工具函数
    """

    def __init__(self,
                 port_name: Optional[str] = None,
                 baudrate: int = 115200,
                 bytesize: int = 8,
                 parity: str = 'N',
                 stopbits: int = 1,
                 timeout: float = 1,
                 minimum_packet_interval: Optional[float] = None,
                 byte_num_of_one_message: int = 7,
                 buffer_size: Optional[int] = None,
                 **kwargs: Any):
        """
        参数初始化

        :param port_name: 串口名称
        :param baudrate: 波特率
        :param bytesize: 数据位
        :param parity: 校验位 ('N', 'E', 'O', 'M', 'S')
        :param stopbits: 停止位
        :param timeout: 超时时间
        :param minimum_packet_interval: 最小包间隔。ascii模式只能保证实际包间隔时间大于仪表的设定值
        :param byte_num_of_one_message: 一个报文的字节数
        :param buffer_size: 串口缓冲区大小
        :param kwargs: 其它参数
        """
        self.port_name = port_name
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.minimum_packet_interval = minimum_packet_interval
        self.byte_num_of_one_message = byte_num_of_one_message

        # 检查额外的参数
        if kwargs:
            raise ValueError('猪头，压根没有这玩意！： {!r}'.format(kwargs))

        # 配置并打开串口
        self.ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=self.bytesize,
            parity=self.parity,
            stopbits=self.stopbits,
            timeout=self.timeout
        )

    def close(self) -> None:
        """显式关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            logger.info("串口已关闭")


    """
    *********************工具函数***********************
    """

    def read_sensor_data(self,
                         standard_message_length: int = 7,
                         report_count: int = 50,
                         chunk_size: int = 512) -> Generator[str, None, None]:
        """
        在ascii通讯模式下读取串口数据：
        1. 首先找到第一个回车符(0D)作为数据同步点
        2. 之后的数据才开始正式接收和解码处理

        :param standard_message_length: 标准通讯模式下，符合标准的默认报文长度（以字节为单位）
        :param report_count: 一次抛出的报文数量限制（已经转换为仪表数值，kg为单位）
        :param chunk_size: 缓冲区大小

        :return: 解码后的报文列表
        """
        if not self.ser.is_open:
            logger.error("串口未打开")
            raise serial.SerialException("串口未打开")


        buffer = bytearray()  # 创建空的字节串
        reports = []  # 创建报文空列表

        while True:                     # 进入数据处理循环
            # 读取新数据并添加到buffer
            if self.ser.in_waiting:     # 如果串口中有数据等待
                # print('串口有数据')
                chunk = self.ser.read(min(self.ser.in_waiting, chunk_size))  # 从等待区和设置的chunk区中，选一个较小的区，进行读取操作
                buffer.extend(chunk)    # 添加到buffer中

            while len(buffer) >= standard_message_length:         # 当buffer超过默认报文长度7
                cr_index = buffer.find(b'\r')       # cr_index作为空格符的索引
                if cr_index == -1 or cr_index < standard_message_length - 1:  # 如果cr_index不存在或小于6。防止串口刚开始接收数据时，数据被截断。
                    break                   # 没有完整报文，退出循环

                valid_data = buffer[:cr_index + 1]
                buffer = buffer[cr_index + 1:]  # 更新buffer

                try:
                    # 将采集到的数据按照ascii编码进行解码，转换为str类型，然后去除首尾的空白符
                    decoded_data = valid_data.decode('ascii').strip()   
                    # reports.append(decoded_data)
                    # print('report添加完毕')
                    # if len(reports) >= report_count:
                        # print('report已抛出')
                        # total_reports += len(reports)       # 采集率计算
                        # yield reports
                    yield decoded_data  # 生成器，每次返回一个报文    
                        # reports = []
                except UnicodeDecodeError as e:
                    logging.error(f"解码错误：{e}，丢弃无效数据")

            # 如果buffer过大，可能表示数据积压，清理旧数据
            if len(buffer) > chunk_size * 2:
                buffer = buffer[-chunk_size:]
                logger.warning("数据积压，清理旧数据")

            # 如果没有足够的报文，短暂等待更多数据到达
            if not reports:
                time.sleep(0.005)  # 等待时间，可根据需要调整


class PipeTransmitter:
    """
    管道传输

    """
    def __init__(self, pipe_path: str = '/tmp/sensor_data_pipe', ascii_model: Optional[AsciiSendModel] = None):
        """
        初始化

        :param pipe_path: 指定的管道路径
        """
        self.pipe_path = pipe_path
        self.fifo = None            # fifo是文件描述符
        self.ascii_model = ascii_model  # 组合方式引用AsciiSendModel

    def open(self):
        """
        打开管道

        :return:
        """
        if not os.path.exists(self.pipe_path):  # 检查指定路径管道是否存在，如果不存在则创建一个新的
            os.mkfifo(self.pipe_path)
            logger.info(f"创建命名管道: {self.pipe_path}")

       # 如果有ascii_model，清理其串口缓冲区
        if self.ascii_model and hasattr(self.ascii_model, 'ser'):
            self.ascii_model.ser.reset_input_buffer()
            self.ascii_model.ser.reset_output_buffer()
            logger.info("串口缓冲区已清理")

        self.fifo = os.open(self.pipe_path, os.O_WRONLY)    # 只写模式打开指定路径管道，将信息赋值给文件描述符，用来代表管道相关信息（开关状态、路径）
        logger.info(f"管道已打开: {self.pipe_path}")          # 日志记录，管道已经打开

    def send_data(self, data: str):
        """
        通过管道发送数据

        :param data: 待发送的数据
        :return:
        """
        if not self.fifo:                       # 根据描述符，检查管道是否打开
            raise RuntimeError("管道未打开")
        # 假设 data 是单个力值的字符串表示，如 "12.34"
        try:
            latest_value = float(data)  # 将字符串转换为浮点数
        except ValueError:
            logging.error(f"无法将数据转换为浮点数: {data}")
            return  # 或者根据需要进行其他处理

        encoded_data = str(latest_value).encode('utf-8')
        packed_length = struct.pack('<I', len(encoded_data))
        try:
            os.write(self.fifo, packed_length + encoded_data) # 通过fifo定位管道，然后一并发送"数据长度+数据内容"。
        except BrokenPipeError:
            logging.error("管道连接断开")
            raise

    def close(self):
        """
        关闭管道

        :return:
        """
        if self.fifo:
            os.close(self.fifo)
            self.fifo = None        # 将描述符归位
            logger.info("管道已关闭")



def run_data_transmission(port_name: str, baudrate: int, pipe_path: str, run_duration: Optional[float] = None):
    ascii_model = None
    pipe_transmitter = None

    try:
        ascii_model = AsciiSendModel(port_name=port_name, baudrate=baudrate)
        pipe_transmitter = PipeTransmitter(pipe_path, ascii_model=ascii_model)
        
        pipe_transmitter.open()

        logger.info("开始数据传输")
        start_time = time.time()
        # buffer = []
        for report in ascii_model.read_sensor_data():  # 积累了指定数量的数据后，返回一次reports。即由ascii_model.read_sensor_data()来触发循环。
                                                        # 每次输出的reports长度理论上是一样的
            # data_to_send = ' '.join(reports)            # 将[str, str, ...]转换为一个连续的单一字符串，以空格为分隔符
            pipe_transmitter.send_data(report)    # 调用send_data发送

            # 或者逐条发送
            # for report in reports:
            #     pipe_transmitter.send_data(report)

            if run_duration and time.time() - start_time > run_duration:
                break

    except Exception as e:
        logger.error(f"发生错误: {e}", exc_info=True)
    finally:
        if ascii_model:
            ascii_model.close()
        if pipe_transmitter:
            pipe_transmitter.close()


class TestInfo:
    """
    专门用于测试的类，里面有各种测试信息相关的函数。
    """
    def __init__(self):
        self.start_time: float = time.time()
        self.total_reports: int = 0              # 采集率初始计数设置0
        self.all_data: List[str] = []

    def add_reports(self, reports: List[str]) -> None:
        """
        添加新的报告数据

        :param reports: 新的报告数据列表
        """
        self.total_reports += len(reports)
        self.all_data.extend(reports)

    def get_results(self) -> Dict[str, Any]:
        """
        获取测试结果

        :return: 包含测试结果的字典
        """
        end_time = time.time()
        duration = end_time - self.start_time
        sampling_rate = self.total_reports / duration if duration > 0 else 0
        return {
            "duration": duration,
            "sampling_rate": sampling_rate,
            "total_reports": self.total_reports,
            "all_data": self.all_data
        }

    def print_results(self) -> None:
        """打印测试结果"""
        results = self.get_results()
        logger.info(f"平均采集率: {results['sampling_rate']:.2f} 个/秒")
        logger.info("所有采集到的数值:")
        for i, data in enumerate(results['all_data'], 1):
            logger.info(f"{i}: {data}")


if __name__ == "__main__":
    # 测试运行，包含测试功能
    # run_ascii_send_model(run_duration=10, enable_test_info=True)
    # 正常运行模式，持续传输数据到管道
    run_data_transmission(port_name='/dev/ttyUSB2', baudrate=115200, pipe_path='/tmp/sensor_data_pipe')