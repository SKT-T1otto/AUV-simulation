from glob import glob
from setuptools import setup

setup(name='crk_auv_demo', version='0.1.0', packages=['crk_auv_demo'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/crk_auv_demo']),
                  ('share/crk_auv_demo', ['package.xml']),
                  ('share/crk_auv_demo/config', glob('config/*')),
                  ('share/crk_auv_demo/worlds', glob('worlds/*'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='CRK Research', maintainer_email='research@example.invalid',
      description='Independent Gazebo Sim 8 single AUV demonstrator', license='Apache-2.0',
      entry_points={'console_scripts': ['controller = crk_auv_demo.controller:main']})
