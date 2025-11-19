from setuptools import setup
from glob import glob
import os

package_name = 'stack_master'

data_files = []
# collect maps files (flatten)
maps=[]
for root, dirs, files in os.walk('maps'):
    for f in files:
        maps.append(os.path.join(root, f))
if maps:
    data_files.append(('share/' + package_name + '/maps', maps))

# collect scripts
scripts = glob('scripts/*')
if scripts:
    data_files.append(('share/' + package_name + '/scripts', scripts))

data_files.append(('share/' + package_name, ['README.md', 'LICENSE']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='you@example.com',
    description='Stack master: maps and processing scripts',
    license='MIT',
)
