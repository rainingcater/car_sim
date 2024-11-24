import datetime
import json

import numpy


class JSONExtEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, (numpy.int_, numpy.intc, numpy.intp, numpy.int8,
                            numpy.int16, numpy.int32, numpy.int64, numpy.uint8,
                            numpy.uint16,numpy.uint32, numpy.uint64)):
            return int(obj)
        elif isinstance(obj, (numpy.float_, numpy.float16, numpy.float32,
                              numpy.float64)):
            return float(obj)
        elif isinstance(obj, (numpy.ndarray,)):
            return {"arr":obj.tolist()}
        elif isinstance(obj, datetime.datetime):
            return obj.strftime('%Y-%m-%d %H:%M:%S')
        elif isinstance(obj, datetime.date):
            return obj.strftime("%Y-%m-%d")
        else:
            return super().default(obj)
        # dd ={}
        # if isinstance(obj, (numpy.ndarray,)):
        #     print(obj.tolist())
        # dd['__class__'] = obj.__class__.__name__
        # dd['__module__'] = obj.__module__
        # dd.update(obj.__dict__)
        # return dd
# ————————————————
# 版权声明：本文为CSDN博主「cswhl」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
# 原文链接：https://blog.csdn.net/cswhl/article/details/111730764

class JSONExtDecoder(json.JSONDecoder):
    def __init__(self):
        super().__init__(object_hook=self.dict_to_obj) # 指定父类JSONDecoder的object_hook转化函数:转化dict为自定义的obj对象

    def dict_to_obj(self,data_dict):
        if isinstance(data_dict,dict):
            if 'arr' in data_dict:
                return numpy.array(data_dict['arr'])
        return data_dict

