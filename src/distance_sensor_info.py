#! /usr/bin/env python

class DistanceSensorInfo: 
    def __init__(self, name, ECHO, TRIGGER, offset):
        self.name = name
        self.ECHO = ECHO
        self.TRIGGER = TRIGGER
        self.offset = offset