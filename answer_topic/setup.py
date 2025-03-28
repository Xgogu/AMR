from setuptools import find_packages, setup

package_name = 'answer_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humble',
    maintainer_email='humble@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'answer_pub = answer_topic.answer_pub:main',
            'answer_sub = answer_topic.answer_sub:main',
        ],
    },
)