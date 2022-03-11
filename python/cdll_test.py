from ctypes import CDLL, CFUNCTYPE, POINTER, c_int, c_uint, pointer, c_ubyte, c_uint8, c_uint32
import os
import sysconfig
import pkg_resources
import site

#library = os.path.abspath("vl53lx_python/bin/vl53lx_python.so")

suffix = '.so' 

_TOF_LIBRARY = CDLL('/home/ubuntu/vl53l3cx-python/bin/vl53lx_python.so')

print(_TOF_LIBRARY)
