import os
from glob import glob
from setuptools import setup

package_name = 'rover_arm_typing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'aruco_keyboard'),
         glob('models/aruco_keyboard/model.*')),
        (os.path.join('share', package_name, 'models', 'aruco_keyboard',
                      'materials', 'textures'),
         glob('models/aruco_keyboard/materials/textures/*.png')),
        (os.path.join('share', package_name, 'models', 'typing_table'),
         glob('models/typing_table/model.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khaja',
    maintainer_email='estyak.ahamedd1139@gmail.com',
    description='Autonomous keyboard-typing stack for the ARCh rover arm',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_detector = rover_arm_typing.keyboard_detector:main',
            'typing_controller = rover_arm_typing.typing_controller:main',
            'key_press_validator = rover_arm_typing.key_press_validator:main',
            'texture_gen = rover_arm_typing.texture_gen:main',
        ],
    },
)
