from setuptools import setup

package_name = 'vla_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/vla_bringup']),
        ('share/vla_bringup', ['package.xml']),
        ('share/vla_bringup/launch', [
            'launch/hospdecor_tb3.launch.py'
        ]),
        ('share/vla_bringup/worlds', [
            'worlds/hospdecor.world'
        ]),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saquib',
    maintainer_email='saquib@todo.todo',
    description='Bringup package',
    license='TODO',

    entry_points={
        'console_scripts': [
            'vision_node = vla_bringup.vision_node:main',
            'object_detector = vla_bringup.object_detector:main',
            'language_grounding = vla_bringup.language_grounding:main',
            'motion_controller = vla_bringup.motion_controller:main',
            'follow_red = vla_bringup.follow_red:main',
            'red_detector = vla_bringup.red_detector:main',
            'red_object_navigator = vla_bringup.red_object_navigator:main',
            'llm_command_node = vla_bringup.llm_command_node:main',
        ],
    },
)

