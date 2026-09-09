from setuptools import setup

setup(
    name='rinbo_monitor', version='0.1.0', packages=['rinbo_monitor'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/rinbo_monitor']),
                ('share/rinbo_monitor', ['package.xml'])],
    package_data={'rinbo_monitor': ['panel.html']},
    install_requires=['setuptools'], zip_safe=False,
    entry_points={'console_scripts': ['rinbo_monitor = rinbo_monitor.server:main']},
)
