from setuptools import setup, Extension

extension = Extension(
    'vl53l3cx_python',
    define_macros=[],
    include_dirs=['.', 'core/inc', 'platform/inc'],
    libraries=[],
    library_dirs=[],
    sources=['api/core/src/vl53lx_api_calibration.c',
             'api/core/src/vl53lx_api_core.c',
             'api/core/src/vl53lx_api_debug.c',
             'api/core/src/vl53lx_api_presets_modes.c',
             'api/core/src/vl53lx_api.c',
             'api/core/src/vl53lx_core_support.c',
             'api/core/src/vl53lx_core.c',
             'api/core/src/vl53lx_dmax.c',
             'api/core/src/vl53lx_hist_algos_gen3.c',
             'api/core/src/vl53lx_hist_algos_gen4.c'
             'api/core/src/vl53lx_hist_char.c',
             'api/core/src/vl53lx_hist_core.c',
             'api/core/src/vl53lx_hist_funcs.c',
             'api/core/src/vl53lx_nvm_debug.c',
             'api/core/src/vl53lx_nvm.c',
             'api/core/src/vl53lx_register_funcs.c',
             'api/core/src/vl53lx_sigma_estimate.c',
             'api/core/src/vl53lx_silocon_core.c',
             'api/core/src/vl53lx_wait.c',
             'api/core/src/vl53lx_xtalk.c',
             'platform/src/vl53lx_platform_init.c',
             'platform/src/vl53lx_platform_ipp.c',
             'platform/src/vl53lx_platform_log.c',
             'platform/src/vl53lx_platform.c',
             'python_lib/vl53lx_python.c'])

setup(name='VL53LX',
      version='0.1',
      description='VL53LX sensor for raspberry PI',
      # author='?',
      # author_email='?',
      url='',
      long_description='''
VL53L3CX sensor for raspberry Pi.
''',
      ext_modules=[extension],
      package_dir={'': 'python'},
      py_modules=['VL53LX'],
      requires=['smbus' or 'smbus2'])