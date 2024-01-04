#!/bin/bash

rosdep update
sudo rosdep install --from-paths src --ignore-src -y

sudo pip install pre-commit
cd src && pre-commit install
