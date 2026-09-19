from pathlib import Path
import os
from setuptools import setup

name = 'fr3_vision_grasp'
here = Path(__file__).resolve().parent
scene = here.parents[2] / 'mujoco_simulations_fr3'
data = [('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml'])]
data.append(('share/' + name, [os.path.relpath(scene.parent / 'LICENSE', here),
                               os.path.relpath(scene.parent / 'THIRD_PARTY_NOTICES.md', here)]))
data.append(('share/' + name + '/scene',
             [os.path.relpath(scene / legal, here) for legal in ('LICENSE', 'NOTICE')]))
for folder in ('launch', 'config'):
    data.append(('share/' + name + '/' + folder,
                 [str(p.relative_to(here)) for p in (here / folder).glob('*') if p.is_file()]))
# Install the existing project assets so the executable never depends on cwd.
for directory in (scene, scene / 'assets'):
    files = [os.path.relpath(p, here) for p in directory.glob('*') if p.suffix.lower() in ('.xml', '.obj', '.stl', '.png')]
    data.append(('share/' + name + '/scene/' + str(directory.relative_to(scene)), files))
setup(name=name, version='0.1.0', packages=[name], data_files=data,
      install_requires=['setuptools'], zip_safe=False,
      entry_points={'console_scripts': [
          'mujoco_bridge = fr3_vision_grasp.bridge:main',
          'color_detector = fr3_vision_grasp.perception:main',
          'pick = fr3_vision_grasp.pick:main',
          'camera_view = fr3_vision_grasp.camera_view:main',
      ]})
