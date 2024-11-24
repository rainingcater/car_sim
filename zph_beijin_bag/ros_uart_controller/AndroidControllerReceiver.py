import json
import os
import socket
import threading
from configparser import NoSectionError, NoOptionError
import random

from util.config_setup import INIConfig
from util.error_util import NoPathFileError
from util.path_util import pathUtil
import time
# redFlag=False
# greenFlag=False
# yellowFlag=False
class AndroidControllerCommMeta(type):
    def __instancecheck__(self, instance):
        return instance.__class__.__name__ in "AndroidControllerCommMeta"
# @UtilTools.Singleton
class AndroidControllerComm(metaclass=AndroidControllerCommMeta):
    __init__flag=False
    def __new__(cls, *args, **kwargs):
        if not hasattr(cls,'_instance'):
            # cls._instance=super().__new__(cls)
            cls._instance=super(AndroidControllerComm,cls).__new__(cls)
            print("创建AndroidControllerComm类对象实例")
        else:
            print("存在AndroidControllerComm类对象实例，使用该单例")
        return cls._instance
# ————————————————
# 版权声明：本文为CSDN博主「伍柳森」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
# 原文链接：https://blog.csdn.net/qq_41248532/article/details/123246471


    def __init__(self, pathfile=os.path.join(pathUtil.getBasePath("exe"), "config", "androidcontrollerconfig.ini")):

        if self.__init__flag==False:
            self.__init__flag=True
            print("执行init")
            self.pathfile=pathfile
            cs=INIConfig(self.pathfile)
            try:
                androidcontrollercomm_sec = dict(cs.readConfigSection("androidcontrollercomm"))
                cs.readConfigValue("androidcontrollercomm","send")
                cs.readConfigValue("androidcontrollercomm","recv")
            except Exception as e:
                if isinstance(e,(NoPathFileError,NoSectionError,NoOptionError)):
                    androidcontrollercomm_dict={
                        "androidcontrollercomm":{
                            "send":('127.0.0.1', 14551),
                            "recv":('0.0.0.0', 14551),
                        }
                    }
                    cs.writeConfigSection(androidcontrollercomm_dict)
                    androidcontrollercomm_sec = dict(cs.readConfigSection("androidcontrollercomm"))

            self.androidcontroller_send_ipport = cs.typeConverter(androidcontrollercomm_sec["send"],flag="backward")
            self.androidcontroller_recv_ipport = cs.typeConverter(androidcontrollercomm_sec["recv"],flag="backward")


            self.__androidcontroller_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.__androidcontroller_socket.bind(self.androidcontroller_recv_ipport)
            self.__androidcontroller_recv_flag = True


    def stop_androidcontroller_receive_thread(self):
        self.__androidcontroller_recv_flag = False
    def stop_loop(self):
        self.stop_androidcontroller_receive_thread()
    """Android部分通信功能"""
    def androidcontroller_receive_info(self, callback):
        threading.Thread(target=self.__androidcontroller_receive_info, args=(callback,self.__androidcontroller_socket)).start()

    def __androidcontroller_receive_info(self, callback,socket):
        while self.__androidcontroller_recv_flag:
            data, send_address = socket.recvfrom(1024)
            # print(f"recvdata:{data}")
            callback(data)
    def androidcontroller_send_info(self,info):
        threading.Thread(target=self.__androidcontroller_send_info, args=(info,)).start()
    def __androidcontroller_send_info(self,info):
        print("send_info:",info)
        self.__androidcontroller_socket.sendto(info, self.androidcontroller_send_ipport)
if __name__=="__main__":
    light=["red","green","yellow"]
    def send_info(accomm):
        while True:
            time.sleep(0.1)
            info=random.choice(light).encode("utf-8")
            accomm.androidcontroller_send_info(info)
    def receive_info_callback(info):
        global redFlag, greenFlag, yellowFlag
        info=info.decode("utf-8")
        # print(f"receive:{info}")
        redFlag=False
        greenFlag=False
        yellowFlag=False
        if "red" == info:
            redFlag=True
        if "green" == info:
            greenFlag=True
        if "yellow" == info:
            yellowFlag=True
    accomm=AndroidControllerComm()
    accomm.androidcontroller_receive_info(receive_info_callback)
    threading.Thread(target=send_info,args=(accomm,)).start()
    while True:
        time.sleep(0.1)
        print(f"redFlag:{redFlag},yellowFlag:{yellowFlag},greenFlag:{greenFlag}")
