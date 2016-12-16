#!/bin/bash

sudo apt-get install python-setuptools

mkdir ~/git
cd ~/git
git clone git://github.com/sympy/sympy.git
cd sympy
sudo python setup.py install

cd ~/git
mkdir cdsousa
cd cdsousa

#must have .ssh key generated, find way to automatically check this
git clone https://github.com/cdsousa/SymPyBotics.git
cd SymPyBotics
sudo python setup.py install

