from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'function_library',          
        'function_library.basic_function'  
    ],
    package_dir={'': 'scripts'},     
)

setup(**d)