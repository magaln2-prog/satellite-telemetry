#!/usr/bin/env python3

#libraries general
import json #library for json files
import os
import shutil #librsry for deleting files and storage
import subprocess #library for running system commands
import time #time library, starts from 1970
from datetime import datetime, timezone #library timestamps UTC
#hardware libraries
import board 
import busio
#environmental sensors. 
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_ltr390
import adafruit_tsl2591
from adafruit_icm20x import ICM20948

#UPS power monitoring
from smbus2 import SMBus



