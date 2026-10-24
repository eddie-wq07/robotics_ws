from pathlib import Path
from setuptools import setup

data = [('share/ament_index/resource_index/packages', ['resource/pen_bringup']),
        ('share/pen_bringup', ['package.xml'])]
for folder in ('web', 'launch'):
    for path in sorted(Path(folder).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc':
            data.append(('share/pen_bringup/' + str(path.parent), [str(path)]))
setup(name='pen_bringup', version='0.1.0', packages=['pen_bringup'],
      data_files=data, install_requires=['setuptools'], zip_safe=False,
      maintainer='Mocha robotics', maintainer_email='robotics@example.com',
      description='Browser and launch integration for the pen pipeline', license='Proprietary',
      tests_require=['pytest'],
      entry_points={'console_scripts': ['web_bridge = pen_bringup.web_bridge:main',
                                       'camera_feed = pen_bringup.camera_feed:main']})
