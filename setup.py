from setuptools import setup

package_name = 'cone_markers'

setup(
    name=package_name,
    version='1.0.0',
    packages=['src'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gameiro',
    maintainer_email='alexandrefgame@mail.com',
    description='Convert Lart ConeArray Message to Markers',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'debug_publisher = src.set_markers:SetMarkers.debug',
            'debug_subscriber = src.get_cones:GetCones.debug',
            'mark_cones = src.__init__:main'
        ],
    },
)