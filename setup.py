import os
from setuptools import setup, find_packages

# Utility function to read the README file.
# Used for the long_description.  It"s nice, because now 1) we have a top level
# README file and 2) it"s easier to type in the README file than to put a raw
# string in below ...
def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


# Courtesy of https://stackoverflow.com/a/36693250
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join("..", path, filename))
    return paths


extra_files = package_files("harosviz")
extra_files.append("*.yaml")
extra_files.append("models/*.yaml")


setup(
    name            = "haros",
    version         = "3.10.5",
    author          = "Andre Santos",
    author_email    = "contact.andre.santos@gmail.com",
    description     = "Static analysis framework for ROS",
    long_description= read("README.md"),
    long_description_content_type = "text/markdown",
    license         = "MIT",
    keywords        = "static-analysis ros",
    url             = "https://github.com/git-afsantos/haros",
    packages        = find_packages(),
    entry_points    = {"console_scripts": ["haros = haros.haros:main"]},
    package_data    = {"haros": extra_files},
    install_requires = [
        "pyyaml",
        "rospkg",
        'python-magic',
        "bonsai-code>=0.5.0,<1.0.0",
        "haros-plugins>=1.0.3,<2.0.0",
        "hpl-specs",
        "ros-type-tokens",
    ],
    extras_require = {},
    zip_safe        = True
)
