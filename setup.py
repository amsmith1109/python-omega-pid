import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="omega-pid",
    version="0.0.1",
    author="Alex M Smith",
    description="Python module for controller Omega Engineering PID controller",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.8',
    py_modules=["omega-pid"],
    package_dir={'':'omega-pid/src'},
    install_requires=[pyserial
                      ]
)