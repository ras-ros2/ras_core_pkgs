#!/usr/bin/env python3


import yaml

def read_yaml(path):
    with open(path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def create_generated_headers_dir():
    import os
    PATH_HPP = "generated_headers"

    if not os.path.exists(PATH_HPP):
        os.makedirs(PATH_HPP)
