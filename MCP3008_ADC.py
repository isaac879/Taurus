#!/usr/bin/env python
#Initial code from: https://learn.adafruit.com/reading-a-analog-in-and-controlling-audio-volume-with-the-raspberry-pi/script
#Modified to work for the Hexapod Scorpion Project
import time
import os
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
DEBUG = 1
GPIO.setwarnings(False)

# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
    if ((adcnum > 7) or (adcnum < 0)):
        return -1
    GPIO.output(cspin, True)

    GPIO.output(clockpin, False)  # start clock low
    GPIO.output(cspin, False)     # bring CS low

    commandout = adcnum
    commandout |= 0x18  # start bit + single-ended bit
    commandout <<= 3    # we only need to send 5 bits here
    for i in range(5):
        if (commandout & 0x80):
            GPIO.output(mosipin, True)
        else:
            GPIO.output(mosipin, False)
        commandout <<= 1
        GPIO.output(clockpin, True)
        GPIO.output(clockpin, False)

    adcout = 0
    # read in one empty bit, one null bit and 10 ADC bits
    for i in range(12):
        GPIO.output(clockpin, True)
        GPIO.output(clockpin, False)
        adcout <<= 1
        if (GPIO.input(misopin)):
                adcout |= 0x1

    GPIO.output(cspin, True)
        
    adcout >>= 1# first bit is 'null' so drop it
    return adcout

#Pins usedccbb
SPICLK = 17
SPIMISO = 27
SPIMOSI = 22
SPICS = 23

# set up the SPI interface pins
GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPICS, GPIO.OUT)

def adcValue(adc):
    # LDR in a potential divider connected cto adc #0
    last_read = 0       # this keeps track of the last potentiometer value

    # read the analog pin
    value = readadc(adc, SPICLK, SPIMOSI, SPIMISO, SPICS)
    #print "ADC %d value = %d" % (adc, value)
    return value
