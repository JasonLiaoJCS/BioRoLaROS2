from setuptools import setup
setup(name='rinbo_control', version='0.1.0', packages=['rinbo_control'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/rinbo_control']),
                  ('share/rinbo_control', ['package.xml'])],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=False,
      entry_points={'console_scripts': ['rinbo_control = rinbo_control.console:main']})
