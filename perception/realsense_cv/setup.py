from setuptools import find_packages, setup

package_name = 'realsense_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/realsense_cv']),
        ('share/realsense_cv', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elkanahung',
    maintainer_email='b09902040@csie.ntu.edu.tw',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'edge_node=realsense_cv.edge_detect:main',
            'hsv_tuning=realsense_cv.hsv_tune:main',
            'marker_node=realsense_cv.marker_detect:main',
            'yolo_node=realsense_cv.yolo_detect:main'
        ],
    },
)
