from setuptools import find_packages, setup

package_name = 'ros2_visao'

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
    maintainer='leticia',
    maintainer_email='leticia.gchavess@gmail.com',
    description='Codigo de visao computacional com OpenCV work2026',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [   
            'talker = ros2_visao.img_pub_node:main',
            'listener = ros2_visao.testedoiscentro:main',
        ],
    },
)
