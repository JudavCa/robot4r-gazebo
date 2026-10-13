from setuptools import find_packages, setup

package_name = 'robot4r_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='judavca',
    maintainer_email='juandavidguerracabrera@gmail.com',
    description='Odometría a partir de las ruedas y comparación con /odom',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometria_ruedas = robot4r_control.odometria_ruedas:main',
        ],
    },
)
