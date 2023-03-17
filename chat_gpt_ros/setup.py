from setuptools import setup

package_name = 'chat_gpt_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuma',
    maintainer_email='yumapine@gmail.com',
    description='chatGPT package for ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chat_gpt_ros = chat_gpt_ros.chat_gpt_ros:main'
        ],
    },
)
