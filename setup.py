from setuptools import setup
from glob import glob

package_name = 'random_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds/', glob('worlds/*.world')),
        ('share/' + package_name + '/maps/', glob('maps/*.yaml')),
        ('share/' + package_name + '/maps/', glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spragunr',
    maintainer_email='nathan.r.sprague@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_nav = random_nav.random_nav:main',
            'nav_demo = random_nav.nav_demo:main'
        ],
    },
)
