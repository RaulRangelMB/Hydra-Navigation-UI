from setuptools import find_packages, setup

package_name = 'hydra_navigation_ui'

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
    maintainer='raul',
    maintainer_email='raulrmb@al.insper.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_calculator_ui = hydra_navigation_ui.sum_ui:main',
            'semantic_navigation_ui = hydra_navigation_ui.semantic_navigation_ui:main',
        ],
    },
)