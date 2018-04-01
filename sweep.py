#!/usr/bin/env python

####################################################################################################
####################################################################################################
##                                                                                                ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub                 ##
## PiStuffing/Quadcopter under GPL for non-commercial application.  Any code derived from         ##
## this should retain this copyright comment.                                                     ##
##                                                                                                ##
## Copyright 2012 - 2018 Andy Baker (Hove) - andy@pistuffing.co.uk                                ##
##                                                                                                ##
####################################################################################################
####################################################################################################


from __future__ import division
from __future__ import with_statement
import signal
import socket
import time
import sys
import getopt
import math
from array import *
import smbus
import select
import os
import io
import logging
import csv
import subprocess
import ctypes
from ctypes.util import find_library
import struct
import serial


####################################################################################################
#
# Functions to lock memory to prevent paging, and move child processes in different process groups
# such that a Ctrl-C / SIGINT to one isn't distributed automatically to all children.
#
####################################################################################################
MCL_CURRENT = 1
MCL_FUTURE  = 2
def mlockall(flags = MCL_CURRENT| MCL_FUTURE):
    libc_name = ctypes.util.find_library("c")
    libc = ctypes.CDLL(libc_name, use_errno=True)
    result = libc.mlockall(flags)
    if result != 0:
        raise Exception("cannot lock memory, errno=%s" % ctypes.get_errno())

def munlockall():
    libc_name = ctypes.util.find_library("c")
    libc = ctypes.CDLL(libc_name, use_errno=True)
    result = libc.munlockall()
    if result != 0:
        raise Exception("cannot lock memory, errno=%s" % ctypes.get_errno())

def Daemonize():
    #-----------------------------------------------------------------------------------------------
    # Discondect child processes so ctrl-C doesn't kill them
    # Increment priority such that Motion is -10, Autopilot and Video are -5, and Sweep and GPS are 0.
    #-----------------------------------------------------------------------------------------------
    os.setpgrp()
    os.nice(5)

    '''
    #AB: ###########################################################################################
    #AB: # Consider here munlockall() to allow paging for lower priority processes i.e. all be main and video
    #AB: ###########################################################################################
    '''


####################################################################################################
#
# Start the Scanse Sweep reading process.
#
####################################################################################################
def SweepProcessor():

    SWEEP_IGNORE_BOUNDARY = 0.5 # 50cm from Sweep central and the prop tips.
    SWEEP_CRITICAL_BOUNDARY = 1.0 # 50cm or less beyond the ignore zone: Hermione's personal space encroached.
    SWEEP_WARNING_BOUNDARY = 1.5 # 50cm or less beyond the critical zone: Pause for thought what to do next.

    sent_critical = False
    warning_distance = 0.0
    previous_degrees = 360.0

    start_time = time.time()
    loops = 0
    samples = 0

    distance = 0.0
    direction = 0.0

    warning_distance = SWEEP_WARNING_BOUNDARY
    warning_radians = 0.0

    with serial.Serial("/dev/ttySWEEP",
                          baudrate = 115200,
                          parity = serial.PARITY_NONE,
                          bytesize = serial.EIGHTBITS,
                          stopbits = serial.STOPBITS_ONE,
                          xonxoff = False,
                          rtscts = False,
                          dsrdtr = False) as sweep:

        try:
            sweep.write("ID\n")
            resp = sweep.readline()

            sweep.write("DS\n")
            resp = sweep.readline()
            assert (len(resp) == 6), "SWEEP: Bad data"

            status = resp[2:4]
            assert status == "00", "SWEEP: Failed %s" % status

            with io.open("/dev/shm/sweep_stream", mode = "wb", buffering = 0) as sweep_fifo:
                log = open("sweep.csv", "wb")
                log.write("angle, distance, x, y\n")

                unpack_format = '=' + 'B' * 7
                unpack_size = struct.calcsize(unpack_format)

                pack_format = '=??ff'

                while True:
                    raw = sweep.read(unpack_size)
                    assert (len(raw) == unpack_size), "Bad data read: %d" % len(raw)

                    #-------------------------------------------------------------------------------
                    # Sweep is spinning at 5Hz sampling at 600Hz.  For large object detection within
                    # SWEEP_CRITICAL range, we can discard 80% of all samples, hopefully providing
                    # more efficient processing and limiting what's sent to the Autopilot.
                    #AB: 600 samples in 1 seconds at 5 circles per seconds = resolution of 3 degrees.
                    #AB: Hence 5 below = 15 degrees checking
                    #-------------------------------------------------------------------------------
                    samples += 1
                    if samples % 5 != 0:
                        continue

                    formatted = struct.unpack(unpack_format, raw)
                    assert (len(formatted) == 7), "Bad data type conversion: %d" % len(formatted)

                    #-------------------------------------------------------------------------------
                    # Read the azimuth and convert to degrees.
                    #-------------------------------------------------------------------------------
                    azimuth_lo = formatted[1]
                    azimuth_hi = formatted[2]
                    angle_int = (azimuth_hi << 8) + azimuth_lo
                    degrees = (angle_int >> 4) + (angle_int & 15) / 16

                    '''
                    #AB: ###########################################################################
                    #AB: # SIX SERIAL REFLECTION FROM THE WIFI ANTENNA TAKES ITS 15CM DISTANCE TO 90CM
                    #AB: # SMACK BANG IN THE CRITICAL ZONE!!! HENCE WE IGNORE THE RANGE OF ANGLES IT
                    #AB: # IS SEEN IN!!
                    #AB: ###########################################################################
                    '''
                    if degrees > 95 and degrees < 97:
                        continue

                    #-------------------------------------------------------------------------------
                    # We only send one warning and critical per loop (~0.2s); warnings happen at the start
                    # of a new loop, criticals immediately.
                    #-------------------------------------------------------------------------------
                    if degrees < previous_degrees:

                        loops += 1
                        output = None

                        #---------------------------------------------------------------------------
                        # Did we get a proximity warning last loop? Send it if so.
                        #---------------------------------------------------------------------------
                        if warning_distance < SWEEP_WARNING_BOUNDARY:
                            output = struct.pack(pack_format, False, True, warning_distance, warning_radians)
                            log_string = "WARNING: %fm @ %f degrees.\n" % (warning_distance , math.degrees(warning_radians) % 360)

                        #---------------------------------------------------------------------------
                        # Have we already sent a critical proximity?  No? Then all's clear.
                        #---------------------------------------------------------------------------
                        '''
                        #AB! This could be improved; there's only a need to send a NONE if the previous loop
                        #AB! sent a WARNING previously, and no WARNING this time round.
                        '''
                        elif not sent_critical:
                            output = struct.pack(pack_format, False, False, 0.0, 0.0)
                            log_string = "PROXIMITY: %fm @ %f degrees.\n" % (distance, degrees % 360)

                        if output != None:
                            sweep_fifo.write(output)
                            log.write(log_string)

                        warning_distance = SWEEP_WARNING_BOUNDARY
                        sent_critical = False

                    previous_degrees = degrees

                    #-------------------------------------------------------------------------------
                    # Sweep rotates ACW = - 360, which when slung underneath equates to CW in the piDrone
                    # frame POV.  Convert to radians and set range to +/- pi radians.
                    #-------------------------------------------------------------------------------
                    radians = -((math.radians(degrees) + math.pi) % (2 * math.pi) - math.pi)

                    #-------------------------------------------------------------------------------
                    # Read the distance and convert to meters.
                    #-------------------------------------------------------------------------------
                    distance_lo = formatted[3]
                    distance_hi = formatted[4]
                    distance = ((distance_hi << 8) + distance_lo) / 100

                    '''
                    #-------------------------------------------------------------------------------
                    # Convert the results to a vector aligned with quad frame.
                    #-------------------------------------------------------------------------------
                    x = distance * math.cos(radians)
                    y = distance * math.sin(radians)

                    log.write("%f, %f, %f, %f\n" % (degrees, distance, x, y))
                    '''

                    #-------------------------------------------------------------------------------
                    # If a reported distance lies inside the danger zone, pass it over to the autopilot
                    # to react to.
                    #-------------------------------------------------------------------------------
                    if distance < SWEEP_IGNORE_BOUNDARY:
                        pass
                    elif distance < SWEEP_CRITICAL_BOUNDARY and not sent_critical:
                        output = struct.pack(pack_format, True, False, distance, radians)
                        sweep_fifo.write(output)
                        log.write("CRITICAL: %fm @ %f degrees.\n" % (distance, degrees % 360))
                        sent_critical = True
                    elif distance < SWEEP_WARNING_BOUNDARY and warning_distance > distance:
                        warning_distance = distance
                        warning_radians = radians

        #-------------------------------------------------------------------------------------------
        # Catch Ctrl-C - the 'with' wrapped around the FIFO should have closed that by here.  Has it?
        #-------------------------------------------------------------------------------------------
        except KeyboardInterrupt as e:
            if not sweep_fifo.closed:
                print "Sweep FIFO not closed! WTF!"

        #-------------------------------------------------------------------------------------------
        # Catch incorrect assumption bugs
        #-------------------------------------------------------------------------------------------
        except AssertionError as e:
            print e

        #-------------------------------------------------------------------------------------------
        # Cleanup regardless otherwise the next run picks up data from this
        #-------------------------------------------------------------------------------------------
        finally:
            sweep.write("DX\n")
            resp = sweep.read()
            log.write("Sweep loops: %d\n" % loops)
            log.write("Time taken: %f\n" % (time.time() - start_time))
            log.write("Samples: %d\n" % samples)
            log.close()


####################################################################################################
#
# Process the Scanse Sweep data.
#
####################################################################################################
class SweepManager():

    def __init__(self):
        #-------------------------------------------------------------------------------------------
        # Setup a shared memory based data stream for the Sweep output
        #-------------------------------------------------------------------------------------------
        os.mkfifo("/dev/shm/sweep_stream")

        self.sweep_process = subprocess.Popen(["python", __file__, "SWEEP"], preexec_fn =  Daemonize)
        while True:
            try:
                self.sweep_fifo = io.open("/dev/shm/sweep_stream", mode="rb")
            except:
                continue
            else:
                break

        self.unpack_format = "=??ff"
        self.unpack_size = struct.calcsize(self.unpack_format)

    def flush(self):
        #-------------------------------------------------------------------------------------------
        # Read what should be the backlog of reads, and return how many there are.
        #-------------------------------------------------------------------------------------------
        raw_bytes = self.sweep_fifo.read(self.unpack_size)
        assert (len(raw_bytes) % self.unpack_size == 0), "Incomplete Sweep data received"
        return int(len(raw_bytes) / self.unpack_size)

    def read(self):
        raw_bytes = self.sweep_fifo.read(self.unpack_size)
        assert (len(raw_bytes) == self.unpack_size), "Incomplete data received from Sweep reader"
        critical, warning, distance, direction = struct.unpack(self.unpack_format, raw_bytes)
        return critical, warning, distance, direction

    def cleanup(self):
        #-------------------------------------------------------------------------------------------
        # Stop the Sweep process if it's still running, and clean up the FIFO.
        #-------------------------------------------------------------------------------------------
        try:
            if self.sweep_process.poll() == None:
                self.sweep_process.send_signal(signal.SIGINT)
                self.sweep_process.wait()
        except KeyboardInterrupt as e:
            pass
        self.sweep_fifo.close()
        os.unlink("/dev/shm/sweep_stream")


    #-----------------------------------------------------------------------------------------------
    # Start up the Sweep and GPS processes if installed
    #-----------------------------------------------------------------------------------------------
    running = True
    try:
        sweep_started = False

        #-------------------------------------------------------------------------------------------
        # Kick off sweep if necessary
        #-------------------------------------------------------------------------------------------
        if sweep_installed:
            sweepp = SweepManager()
            sweep_fd = sweepp.sweep_fifo.fileno()
            poll.register(sweep_fd, select.POLLIN | select.POLLPRI)
            sweep_started = True

    except:
        #-------------------------------------------------------------------------------------------
        # By setting this, we drop through the big while running the flight plans, and immediately
        # send a finished message to the autopilot processor
        #-------------------------------------------------------------------------------------------
        running = False

    #-----------------------------------------------------------------------------------------------
    # Loop for the period of the flight defined by the flight plan contents
    #-----------------------------------------------------------------------------------------------
    pack_format = '=3f20s?' # edx, edy and edz float targets, string state name, bool running
    log = open("autopilot.log", "wb")

    #-------------------------------------------------------------------------------------------
    # Off we go!
    #-------------------------------------------------------------------------------------------


        except KeyboardInterrupt as e:
            #---------------------------------------------------------------------------------------
            # The motion processor is finished with us, we should too, and we have by breaking out of
            # the with.
            #---------------------------------------------------------------------------------------
            if not autopilot_fifo.closed:
                print "Autopilot FIFO not closed! WTF!"

        except Exception as e:
            log.write("AP: UNIDENTIFIED EXCEPTION: %s\n" % e)

        finally:
            #---------------------------------------------------------------------------------------
            # Cleanup Sweep if installed.
            #---------------------------------------------------------------------------------------
            if sweep_installed and sweep_started:
                print "Stopping Sweep... ",
                sweepp.cleanup()
                poll.unregister(sweep_fd)
                print "stopped."

    log.close()



        #-------------------------------------------------------------------------------------------
        # Unregister poll registrars
        #-------------------------------------------------------------------------------------------
        if self.autopilot_installed:
            poll.unregister(autopilot_fd)

        if self.camera_installed:
            poll.unregister(video_fd)



    ################################################################################################
    #
    # Shutdown triggered by early Ctrl-C or end of script
    #
    ################################################################################################
    def shutdown(self):

        #-------------------------------------------------------------------------------------------
        # Close stats logging file.
        #-------------------------------------------------------------------------------------------
        file_handler.close()

        #-------------------------------------------------------------------------------------------
        # Unlock memory we've used from RAM
        #-------------------------------------------------------------------------------------------
        munlockall()

        sys.exit(0)





####################################################################################################
# If we've been called directly, this is the spawned video, GPS, Sweep or autopilot process or a
# misinformed user trying to start the code.
####################################################################################################
if __name__ == '__main__':
    if len(sys.argv) >= 2:

        #-------------------------------------------------------------------------------------------
        # Start the process recording Sweep
        #-------------------------------------------------------------------------------------------
        elif sys.argv[1] == "SWEEP":
            assert (len(sys.argv) == 2), "Bad parameters for Sweep"
            SweepProcessor()
        else:
            assert (False), "Invalid process request."
    else:
        print "If you're trying to run me, use 'sudo python ./qc.py'"
