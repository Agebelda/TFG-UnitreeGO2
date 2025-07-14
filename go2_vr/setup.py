from setuptools import setup, find_packages

package_name = 'go2_vr'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py', 'launch/pc.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alejandro Gea Belda',
    maintainer_email='alejandrogeabelda@gmail.com',
    description='Sistema de realidad virtual para el control inmersivo de un perro robótico',
    license='GNU General Public License v3 (GPLv3)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_publisher = go2_vr.robot.stereo_publisher:main',
            'cmd_vel_request = go2_vr.robot.cmd_vel_request:main',
            'image_compressor_left = go2_vr.pc.image_compressor_left:main',
            'image_compressor_right = go2_vr.pc.image_compressor_right:main',
        ],
    },
)
