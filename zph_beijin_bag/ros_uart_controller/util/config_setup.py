import configparser
import os

import numpy as np
import json

from util.JSONExtCoder import JSONExtDecoder, JSONExtEncoder
from util.error_util import NoPathFileError


class INIConfig:
    """配置项"""

    def __init__(self,pathfile):
        # self.pathfile=os.path.abspath(pathfile)
        self.pathfile=pathfile
        self.config = configparser.RawConfigParser()
    def readConfigSection(self,section):
        if not os.path.exists(self.pathfile):
            raise NoPathFileError("文件:%s不存在"%self.pathfile)
        ini_r=open(self.pathfile,"r")
        self.config.read_file(ini_r)
        return self.config.items(section)
    def readConfigValue(self,section,key):
        if not os.path.exists(self.pathfile):
            raise NoPathFileError("文件:%s不存在"%self.pathfile)
        ini_r=open(self.pathfile,"r")
        self.config.read_file(ini_r)
        obj=self.typeConverter(self.config.get(section,key),flag="backward")
        return obj
    def writeConfigSection(self,kv_sec_dict):
        if not isinstance(kv_sec_dict,dict):
            raise TypeError("Section数据类型不是Dict，错误")
        kv_sec_dict=dict(kv_sec_dict)
        pathstr=os.path.split(self.pathfile)[0]
        if not os.path.exists(pathstr):
            print('创建路径：%s' % (pathstr))
            os.makedirs(pathstr)
        if not os.path.exists(self.pathfile):
            ini_w=open(self.pathfile,"w")
            for sec_k,sec_v in kv_sec_dict.items():
                self.config.add_section(sec_k)
                for k,v in sec_v.items():
                    v=self.typeConverter(v)
                    self.config.set(sec_k,k,v)
            self.config.write(ini_w)
            ini_w.close()
        else:
            ini_r=open(self.pathfile,"r")
            self.config.read_file(ini_r)

            for sec_k,sec_v in kv_sec_dict.items():
                if sec_k not in self.config.sections():
                    self.config.add_section(sec_k)
                for k,v in sec_v.items():
                    v=self.typeConverter(v)
                    self.config.set(sec_k,k,v)
            ini_w=open(self.pathfile,"w")
            self.config.write(ini_w)
            ini_w.close()
            ini_r.close()
    def typeConverter(self,v,flag="forward"):
        if flag=="forward":
            if isinstance(v,np.ndarray):
                return json.dumps({"arr":v.tolist()})
            else:
                return f'{v}'
        elif flag=="backward":
            try:
                obj=eval(v)
            except NameError as e:
                print(e,end=",")
                print("尝试返回该字段")
                return v
            if isinstance(obj,dict) and "arr" in obj:
                return np.array(obj["arr"])
            else:
                return obj
        else:
            return v
    def writeConfigValue(self,section,k,v):
        pathstr=os.path.split(self.pathfile)[0]
        if not os.path.exists(pathstr):
            print('创建路径：%s' % (pathstr))
            os.makedirs(pathstr)
        if not os.path.exists(self.pathfile):
            ini_w=open(self.pathfile,"w")
            self.config.add_section(section)
            self.config.set(section,k, f"{v}")
            self.config.write(ini_w)
        else:
            ini_r=open(self.pathfile,"r")
            self.config.read_file(ini_r)
            if section not in self.config.sections():
                self.config.add_section(section)
            self.config.set(section,k,v)
            ini_w=open(self.pathfile,"w")
            self.config.write(ini_w)
            ini_w.close()
            ini_r.close()
class JSONConfig:
    @staticmethod
    def readJSONConfig(pathfile,cls=JSONExtDecoder):
        try:
            with open(pathfile,'r',encoding="utf8") as json_file:
                json_obj=json.load(json_file,cls=cls)
        except Exception as e:
            print("无法读取文件：%s\n报错信息：%s\n处理方式：return {}"%(pathfile,e))
            json_obj={}
        return json_obj
    @staticmethod
    def writeJSONConfig(json_obj,pathfile,cls=JSONExtEncoder):
        with open(pathfile,'w',encoding="utf8") as json_file:
            print("写入文件:%s,内容:%s"%(json_file,json_obj),end="")
            json.dump(
                json_obj,
                json_file,
                cls=cls,#json编码器
                ensure_ascii=False,#支持中文
                indent=4#格式化
            )
            print(",完成")

if __name__ == '__main__':
    test_dict={"sec1":{"aa": {6,7,8,9},"bb":np.array([[1,2],[3,4],[5,6]])},"sec2":{"cc":[1,2,3,4],"dd":"ahaha"}}
    settings=INIConfig(os.path.join("../src/test", "aa.txt"))
    settings.writeConfigSection(test_dict)
    print(settings.readConfigSection("sec1"))
    print(settings.readConfigValue("sec1","aa"))
    print(settings.readConfigValue("sec1","bb"))
    print(settings.readConfigValue("sec2","cc"))
    print(settings.readConfigValue("sec2","dd"))
    try:
        print(settings.readConfigValue("sec2","aaa"))
        print(settings.readConfigValue("sec3","ee"))
    except Exception as e:
        if isinstance(e, configparser.NoOptionError):
            settings.writeConfigValue("sec2","aaa","aaaaaaaa")
            print(settings.readConfigValue("sec2","aaa"))
        if isinstance(e, configparser.NoSectionError):
            sec={"sec3":{"ee":"0%N4XsTdT7n6FIRw@K7SGWvGZYJDDkN#"}}
            settings.writeConfigSection(sec)
            print(settings.readConfigValue("sec3","ee"))
