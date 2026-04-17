from setuptools import find_packages, setup


package_name = 'rover_autonomy'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Codex',
    maintainer_email='team@example.com',
    description='Lean autonomy nodes for the Round 3 rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_processor = rover_autonomy.vision_processor:main',
            'state_machine_controller = rover_autonomy.state_machine_controller:main',
            'safety_controller = rover_autonomy.safety_controller:main',
        ],
    },
)
