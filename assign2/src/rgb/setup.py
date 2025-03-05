from setuptools import find_packages, setup

package_name = 'rgb'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'cv_bridge'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description=f'oakd {package_name} camera',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'image = {package_name}.get_image:main',
            f'blob_detect = {package_name}.blob_detect:main',
            f'move_to_color = {package_name}.move_to_color:main'
        ],
    },
)
