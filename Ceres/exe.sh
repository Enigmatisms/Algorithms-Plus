#!/bin/bash

cd build
cmake .. && make -j8

./Task1

python ../plot.py `./Task2`
