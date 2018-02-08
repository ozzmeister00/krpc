#!/usr/bin/env python

import setuptools

name = 'ozzybear_krpc'


setuptools.setup(
    name=name,
    version='0.1',
    author="Matt Oztalay and Tyler Jachetta",
    author_email="me@tylerjachetta.net",
    url="www.hemaalliance.com",
    description="the dumb name is Tyler's fault",
    long_description="todo",
    requires=['krpc'],
    license="MIT License",
    packages=setuptools.find_packages(),
    data_files=[],
)
