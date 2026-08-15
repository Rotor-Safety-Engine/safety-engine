"""
Rotor Safety Engine - Real-time physics safety engine for robotics and VLA systems.
"""

from setuptools import setup, find_packages
import os

# Read version from source
version = "1.1.0"

# Read README
here = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(here, "README.md"), encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="rotor-safety-engine",
    version=version,
    description="Real-time physics safety engine for robotics and VLA systems",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Rotor Dynamics",
    author_email="contact@rotor-dynamics.ai",
    url="https://github.com/rotor-dynamics/safety-engine",
    license="MIT",
    packages=find_packages("src"),
    package_dir={"": "src"},
    py_modules=["safety_engine"],
    python_requires=">=3.7",
    install_requires=[],
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: System :: Hardware",
    ],
    keywords=[
        "robotics", "safety", "vla", "physics", "embedded",
        "vision-language-action", "collaboration-robot",
        "iso-10218", "iso-15066",
    ],
    zip_safe=True,
)
