from setuptools import setup

setup(
    name='kick_calibration', version='0.1.0', packages=['kick_calibration'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/kick_calibration']),
                ('share/kick_calibration', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Raphinha2006', maintainer_email='3055450382@qq.com',
    description='Terminal-driven native K1 kick calibration and raw recording',
    license='Apache-2.0',
)
