from setuptools import find_packages, setup

package_name = 'description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),  # Add this line
        ('share/' + package_name + '/urdf', ['urdf/new_arm_assembly.urdf']),
        ('share/' + package_name + '/meshes', ['meshes/base_link.STL', 'meshes/hand_link.STL', 'meshes/link_1.STL', 'meshes/link_2.STL']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='indy03',
    maintainer_email='kandapah@ualberta.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
