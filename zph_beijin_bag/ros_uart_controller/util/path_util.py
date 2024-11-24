#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import sys, os
class pathUtil:
    #参考示例代码网址：https://pyinstaller.org/en/v5.11.0/runtime-information.html
    @staticmethod
    def getBasePath(path_type="tmp"):
        """
        获取基路径
        :param path_type: 路径类型：
                1、tmp：非Frozen模式下，相对于temp/_MEIXXXXXX路径；
                2、exe：非Frozen模式下，相对于入口文件计算基路径；
                3、lib：Frozen模式下，相对于当前文件计算基路径；
        :return: 基路径
        """
        frozen = 'not'
        if getattr(sys, 'frozen', False):
            # we are running in a PyInstaller bundle
            if path_type=="tmp":#相对于temp/_MEIXXXXXX路径
                frozen = 'ever so'
                bundle_dir = sys._MEIPASS
            elif path_type=="exe":#相对于打包后exe文件路径
                bundle_dir = os.path.dirname(os.path.dirname(sys.executable))#相对于dist路径下exe文件计算根路径
            elif path_type=="lib":
                raise Exception("非Frozen模型下无法设置路径类型为lib！")
            else:
                raise Exception("路径类型设置错误！")
        else:
            # we are running in a normal Python process
            if path_type=="lib":
                bundle_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))#相对于当前文件计算根路径
            elif path_type in ["exe","tmp"]:
                bundle_dir = os.path.dirname(os.path.dirname(sys.argv[0]))#相对于入口文件计算根路径
            else:
                raise Exception("路径类型设置错误！")
        # print( 'we are',frozen,'frozen')
        # print( 'bundle dir is', bundle_dir )
        # print( 'sys.argv[0] is', sys.argv[0] )
        # print( 'sys.executable is', sys.executable )
        # print( 'abspath(__file__) is', os.path.abspath(__file__) )
        # print( 'os.getcwd is', os.getcwd(),'\n' )
        return bundle_dir

if __name__ == '__main__':
    pathUtil.getBasePath(path_type="exe")