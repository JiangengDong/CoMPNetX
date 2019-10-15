#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/AtlasMPNet')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    AtlasMPNet = RaveCreatePlanner(env,'AtlasMPNet')
    print AtlasMPNet.SendCommand('help')
finally:
    RaveDestroy()
