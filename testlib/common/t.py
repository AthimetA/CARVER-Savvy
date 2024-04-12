#!/usr/bin/python3
import sys
import os
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(path)
import numpy as np
import rclpy
from rclpy.node import Node

def test2():
    print(f'Hello from test() in {__file__}!')