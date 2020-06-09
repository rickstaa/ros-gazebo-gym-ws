"""A setup file for the panda_training python scripts. This setup file is used to
install all the python dependencies that are needed to successfully run these RL
training scripts.
"""

# Future Imports
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

# Standard library imports
import logging
import os
from setuptools import setup, find_packages
from setuptools.command.develop import develop
from setuptools.command.install import install
import subprocess
import sys
import re
from distutils.sysconfig import get_python_lib

# Get the relative path for including the data files
relative_site_packages = get_python_lib().split(sys.prefix + os.sep)[1]
date_files_relative_path = os.path.join(relative_site_packages, "panda_openai_sim")

# General setup.py parameters
TF_MAX_VERSION = "1.15"  # NOTE: Currently the examples used tensorflow 1.

# Package requirements
requirements = ["numpy", "stable_baselines", "gym"]

# Set up logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


#################################################
# Setup functions ###############################
#################################################
def get_tf_dep():
    """Check whether or not the Nvidia driver and GPUs are available and add the
    corresponding Tensorflow dependency."""

    # Find the right tensorflow version to install
    tf_dep = "tensorflow<={}".format(TF_MAX_VERSION)
    try:
        gpus = (
            subprocess.check_output(
                ["nvidia-smi", "--query-gpu=gpu_name", "--format=csv"]
            )
            .decode()
            .strip()
            .split("\n")[1:]
        )
        if len(gpus) > 0:
            tf_dep = "tensorflow-gpu<={}".format(TF_MAX_VERSION)
        else:
            no_device_msg = (
                "Found Nvidia device driver but no"
                " devices...installing Tensorflow for CPU."
            )
            logger.warning(no_device_msg)
    except OSError:
        no_driver_msg = (
            "Could not find Nvidia device driver...installing" " Tensorflow for CPU."
        )
        logger.warning(no_driver_msg)
    return tf_dep


#################################################
# Setup classes #################################
#################################################
class DevelopCmd(develop):
    """Overload :py:class:`setuptools.command.develop.develop` class."""

    user_options_custom = [
        ("docker", None, "installing in Docker"),
        ("sing", None, "installing in Singularity"),
    ]
    user_options = getattr(develop, "user_options", []) + user_options_custom

    def initialize_options(self):
        develop.initialize_options(self)

        # Initialize options.
        self.docker = False
        self.sing = False

    def finalize_options(self):
        develop.finalize_options(self)

    def run(self):
        """Overload the :py:mh:`setuptools.command.develop.develop.run` method."""

        # Install Tensorflow dependency.
        if not self.docker and not self.sing:
            tf_dep = get_tf_dep()
            subprocess.Popen([sys.executable, "-m", "pip", "install", tf_dep]).wait()
        else:

            # If we're using a Docker or singularity container, the right
            # tensorflow version is specified in the recipe file. This is
            # done since there is no way to check for CUDA/AMD GPU's at
            # container build time.
            skip_tf_msg = (
                "Omitting Tensorflow dependency because of Docker" " installation."
            )
            logger.warning(skip_tf_msg)

        # Run installation.
        develop.run(self)


class InstallCmd(install, object):
    """Overload :py:class:`setuptools.command.install.install` class"""

    # Add extra user arguments
    user_options_custom = [
        ("docker", None, "installing in Docker"),
        ("sing", None, "Installing in Singularity"),
    ]
    user_options = getattr(install, "user_options", []) + user_options_custom

    def initialize_options(self):
        """Initialize extra argument options."""
        install.initialize_options(self)

        # Initialize options
        self.docker = False
        self.sing = False

    def finalize_options(self):
        """Set extra argument options."""
        install.finalize_options(self)

    def run(self):
        """Overload the :py:meth:`setuptools.command.install.install.run` method."""

        # Install Tensorflow dependency.
        if any(["tensorflow" in item for item in self.distribution.install_requires]):
            if not self.docker and not self.sing:
                tf_dep = get_tf_dep()
                subprocess.Popen(
                    [sys.executable, "-m", "pip", "install", tf_dep]
                ).wait()
            else:

                # If we're using a Docker or singularity container, the right
                # tensorflow version is specified in the recipe file. This is
                # done since there is no way to check for CUDA/AMD GPU's at
                # container build time.
                skip_tf_msg = (
                    "Omitting Tensorflow dependency because of Docker" " installation."
                )
                logger.warning(skip_tf_msg)

        # Run parent run method
        install.run(self)


#################################################
# Setup script ##################################
#################################################

# Get current package version
# NOTE: We use the version of the panda_openai_sim package.
__version__ = re.sub(
    r"[^\d.]",
    "",
    open(
        os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                "..",
                "panda_openai_sim/version.py",
            )
        )
    ).read(),
)

# Parse readme.md
with open("README.md") as f:
    readme = f.read()

# Run python setup
setup(
    name="panda_training",
    version=__version__,
    description=(
        "A python package with several example RL training scripts for the"
        "panda_openai_sim package."
    ),
    long_description=readme,
    long_description_content_type="text/markdown",
    author="Rick Staa",
    author_email="rick.staa@outlook.com",
    license="Rick Staa copyright",
    url="https://github.com/rickstaa/panda_openai_sim",
    keywords="ros, rl, panda, openai gym",
    classifiers=[
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Natural Language :: English",
        "Topic :: Scientific/Engineering",
    ],
    packages=["panda_training"],
    install_requires=requirements,
    extras_require={},
    include_package_data=True,
    data_files=[(date_files_relative_path, ["README.md"])],
    cmdclass={"install": InstallCmd, "develop": DevelopCmd},
)
