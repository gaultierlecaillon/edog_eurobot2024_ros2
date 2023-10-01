#!/usr/bin/python3
import time
import board
import neopixel
from random import randint
import sys

NUM_PIXEL = 6 + 6 + 15 + 15 # rEyes lEyes strip


class Leds:


    def __init__(self):
        self.pixels = neopixel.NeoPixel(board.D18, NUM_PIXEL)
        self.arr = sys.argv[1].split(',')
        print(self.arr)

    def standBy(self):
        for x in range(0, NUM_PIXEL):
            self.pixels[x] = (250, 0, 250)

        for led in self.arr:
            print("ddddd", led)
            self.pixels[int(led)] = (250, 0, 0)


Leds = Leds();
Leds.standBy();
