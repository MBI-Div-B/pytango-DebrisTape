from setuptools import setup, find_packages

setup(
    name="tangods_debristape",
    version="0.0.2",
    description="Tango device for DebrisTape",
    author="Daniel Schick",
    author_email="dschick@mbi-berlin.de",
    python_requires=">=3.6",
    entry_points={"console_scripts": ["DebrisTape = tangods_debristape:main"]},
    license="MIT",
    packages=["tangods_debristape"],
    install_requires=[
        "pytango",
        "w1thermsensor",
    ],
    url="https://github.com/MBI-Div-b/pytango-DebrisTape",
    keywords=[
        "tango device",
        "tango",
        "pytango",
    ],
)
