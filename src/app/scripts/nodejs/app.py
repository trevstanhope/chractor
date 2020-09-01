#!/usr/bin/env python 

"""
Chractor
Embedded system for monitoring traction, suspension, and engine
Designed for Jeep Cherokee XJ
"""

__author__ = 'Trevor Stanhope'
__version__ = '0.0.0.1'

# Modules
from cherrypy.process.plugins import Monitor
import json
import cherrypy
from cherrypy import tools
from pymongo import MongoClient
from bson import json_util
from pymongo import MongoClient
import numpy as np
import sys, os
import serial
import binascii
import random
import cvme
from datetime import datetime

class Logger:
    """ Logging system """
    def __init__(self, fname, ftype=".csv"):
        self.file = open(os.path.join("logs", fname + ftype), 'w')
    def write_headers(self, d):
        if d is not None:
            self.file.write(','.join([str(k) for k,v in d.iteritems()] + ['\n']))
    def insert_data(self, d):
        if d is not None:
            self.file.write(','.join([str(v) for k,v in d.iteritems()] + ['\n']))

class WatchDog:
    def __init__(self):
        self.time = datetime.datetime.now()

class App:
    """ Web Interface """
    def __init__(self, config_file):
        try:
            with open(config_file) as cfg:
                self.config = json.loads(cfg.read())
            self.session_key = binascii.b2a_hex(os.urandom(self.config['SESSION_KEY_LENGTH']))
            self.latest_data = {}

            # Mongo
            try:
                self.mongo_client = MongoClient()
                self.db = self.mongo_client[self.config['MONGO_NAME']]
                self.session = self.db[self.session_key]
            except:
                self.print_error('MONGO', e)

            # OBD
            try:
                self.gateway = Gateway()
                self.gateway.attach()
                self.gateway.get_device()
            except Exception as e:
                self.print_error('Gateway failed', e)

            # CLORB
            try:
                self.camera = cvme.CLORB()
            except:
                self.print_error('CLORB failed', e)

            # Logger
            try:
                self.log = Logger(self.session_key)
            except Exception as e:
                self.print_error('LOGGER', e)

            # Scheduled Tasks
            try:
                Monitor(cherrypy.engine, self.listen, frequency=self.config["POLLING_FREQ"]).subscribe()
                #Monitor(cherrypy.engine, self.capture, frequency=self.config["CAPTURE_FREQ"]).subscribe()
            except Exception as e:
                raise e
        except Exception as e:
            self.print_error('SYSTEM', e)
    
    def print_error(self, subsystem, e):
        print datetime.strftime(datetime.now(), "[%d/%b/%Y:%H:%H:%S]") + ' ' + subsystem + ' ' + str(e)
    
    ## Task Functions
    def listen(self):
        try:
            self.latest_data = self.gateway.poll()
            #self.log.insert_data(self.latest_data)
        except Exception as e:
            print str(e)

    def capture(self):
        try:
            self.camera.get_latest()
        except Exception as e:
            print str(e)

    ## Server Handlers
    @cherrypy.expose 
    def index(self, index_file="index.html"):
        path = os.path.join(self.config['CHERRYPY_PATH'], index_file)
        with open(path) as html: 
            return html.read()

    @cherrypy.expose
    @tools.json_out()
    @cherrypy.tools.accept(media='application/json')
    def default(self, *args, **kwargs):
        """ This function the API, handle API Requests """
        try:
            if args[0] == 'api':
                self.print_error("JSON", "Caught request from app API")
                print self.latest_data
                return self.latest_data
        except Exception as e:
            raise e
        return {}

class Gateway:
    def __init__(self, debug=False):
        """ Gateway """
        self.port = None
        self.debug = debug

        # JSON Data to be transmitted to the app over the web API
        self.json_data = {
            "slip" : 0,
            "gear_ratio" : 0, # change
            "acceleration" : 0, # change
            "cvt_pct" : 0, # change
            "engine_rpm" : 0,
            "gear" : "P",
            "trans_temp" : 0,
            "throttle" : 0,
            "pitch" : 0,
            "roll" : 0,
            "battery_voltage" : 0,
            "uptime" : 0, # change
            "lock" : 0, # change
            "engine_temp" : 0,
            "oil" : 0,
            "load" : 0,
            "suspension_flex" : 0,
            "transfer_case" : "2H",
            "ground_speed" : 0,
            "hours" : 0
        }

    def attach(self, device_classes=['/dev/ttyUSB','/dev/ttyACM'], attempts=5, baud=38400):
        """  Connect to OBD """
        if self.port is None:
            for i in range(attempts):
                for dev in device_classes:
                    self.dev_id = None
                    try:
                        self.dev_id = dev + str(i)
                        self.port = serial.Serial(self.dev_id, baud) # set self.port to the located device
                        return True
                    except Exception as e:
                        self.port = None
            if self.port is None:
                raise Exception("Could not locate OBD device!")
        else:
            raise Exception("Port already attached!")
    
    def poll(self):
        """
        Grab the latest OBDII data via the Gateway
        Display requires the following keys:
        See src/app/display.tsc for required key-values
        """
        if self.port is not None:
            try:
                string = self.port.readline()
            except:
                raise Exception("Failed to read from OBD")
            try:
                msg = json.loads(string) # parse JSON
                #!TODO
            except:
                raise Exception("Failed to parse message as JSON!")
        return self.json_data

    def print_msg(self, msg):
        print datetime.strftime(datetime.now(), "[%d/%b/%Y:%H:%H:%S]") + ' CANBUS ' + str(msg)

    def checksum(self, d, mod=256, decimals=2):
        """ Returns: mod N checksum """
        chksum = 0
        d = {k.encode('ascii'): v for (k, v) in d.iteritems()}
        for k,v in d.iteritems():
            if v is float:
                d[k] = round(v,decimals)
        s = str(d)
        r = s.replace(' ', '').replace('\'', '\"')
        for i in r:
            chksum += ord(i)
        return chksum % mod

    def get_device(self):
        """ Returns the device ID """
        if self.dev_id is not None:
            return self.dev_id
        else:
            return False

if __name__ == '__main__':
    config_file = "settings.json"
    app = App(config_file)
    cherrypy.server.socket_host = "0.0.0.0"
    cherrypy.server.socket_port = 8080
    currdir = os.path.dirname(os.path.abspath(__file__))
    conf = {
        '/': {'tools.staticdir.on':True, 'tools.staticdir.dir':os.path.join(currdir,app.config['CHERRYPY_PATH'])},
    }
    cherrypy.quickstart(app, '/', config=conf)
