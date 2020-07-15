#!/bin/bash

if [ ! -d cvss ]; then
    mkdir cvss
fi

if [ ! -f cvss/data.csv ]; then
    rm cvss/data.csv
fi

cd RAV/

for distance in 105 ; do #, 90, 95, 100, 105); do
    for angle in 30 40 45; do #30 40 45); do
        ./Task1 $distance $angle
    done
done

echo "Test process completed."