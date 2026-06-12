from setuptools import find_packages, setup

package_name = 'fuzzy_control'

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
    maintainer='ares',
    maintainer_email='ares@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mamdani_v1 = fuzzy_control.line_mamdani_v1:main',
            'mamdani_v2 = fuzzy_control.line_mamdani_v2:main',
            'mamdani_aprox = fuzzy_control.line_aproximated:main',
            'sugeno_0 = fuzzy_control.line_sugeno_0:main',
            'sugeno_1 = fuzzy_control.line_sugeno_1:main',
            'sugeno_pd= fuzzy_control.line_sugeno_pd:main',
        ],
    },
)
