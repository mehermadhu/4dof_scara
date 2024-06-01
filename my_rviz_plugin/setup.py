from setuptools import setup

package_name = 'my_rviz_plugin'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin_description.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='RViz plugin to publish joint values',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_values_panel = my_rviz_plugin.joint_values_panel:main',
        ],
    },
)
