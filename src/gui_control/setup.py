from setuptools import find_packages, setup

package_name = 'gui_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boinggboi',
    maintainer_email='nattmatt09@gmail.com',
    description='GUI control package for Subsystem 3 - Interaction and Execution',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gui_node = gui_control.gui_node:main',
        ],
    },
)