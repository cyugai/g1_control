from setuptools import find_packages, setup

package_name = 'g1_mujoco_sim'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bth',
    maintainer_email='bth@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],},
    entry_points={
        'console_scripts': [
        "g1_sim=g1_mujoco_sim.g1_sim:main",
        "joy=g1_mujoco_sim.joy:main",
        "teleop=g1_mujoco_sim.teleop:main",     
        ],
    },
)
