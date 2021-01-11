from setuptools import setup
from glob import glob

package_name = 'iac_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (['share/' + package_name], ['resources/RtiSCADE_DS_Controller_ego1.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suraj',
    maintainer_email='suraj.rathi00@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    # tests_require=['pytest'],
    # entry_points={
    #     'console_scripts': [
    #         'main = iac_planner.main:main',
    #     ],
    # },
)
