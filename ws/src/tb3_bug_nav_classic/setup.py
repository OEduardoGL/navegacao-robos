from setuptools import setup

package_name = 'tb3_bug_nav_classic'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bug2_classic.launch.py',
            'launch/tangent_classic.launch.py',
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/bug_world.world',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Bug2 and TangentBug with Gazebo Classic',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bug2_node = tb3_bug_nav_classic.bug2_node:main',
            'tangent_bug_node = tb3_bug_nav_classic.tangent_bug_node:main',
        ],
    },
)
