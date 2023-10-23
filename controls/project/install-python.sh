#! /bin/bash

pyenv shell 3.7
poetry env use $(which python3)
poetry install
poetry add carla==0.9.13
