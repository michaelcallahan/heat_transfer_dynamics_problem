from setuptools import setup, find_packages

setup(
    name='heat_transfer_simulation',
    version='0.1.0',
    description='Simulation of heat transfer between a steel ball and an aluminum yoke with a heater blower input.',
    author='Michael Callahan',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'scipy',
        'matplotlib',
        'control'
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)
