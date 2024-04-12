#!/usr/bin/python3
import sys
import os
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(path)

def test():
    print(f'Hello from test() in {__file__}!')