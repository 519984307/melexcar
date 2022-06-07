#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import serial
import socket
import time
from multiprocessing import Process, Manager

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 20
        self.ini =True
        self.sensores = None
       # with Manager() as manager:
        manager=Manager()
        # creamos dicionario
        self.sensores = manager.dict()
        # creamos procesos de comunicación por socket
        p = Process(target=self.com_socket, args=('192.168.50.40', 2001, self.sensores))
        p2 = Process(target=self.com_socket, args=('192.168.50.41', 2001, self.sensores))
        p3 = Process(target=self.com_socket, args=('192.168.50.42', 2002, self.sensores))
        # self.p2 = Process(target=self.com_socket, args=('192', 2001, self.sensores))
        # arrancamos los procesos
        p.start()
        p2.start()
        p3.start()


        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 24
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):


        return True


    @QtCore.Slot()
    def compute(self):
        '''
        with Manager() as manager:
            # creamos dicionario

            if self.ini:
                self.sensores = manager.dict()
                # creamos procesos de comunicación por socket
                p = Process(target=self.com_socket, args=('192.168.50.43', 2001, self.sensores))
                # self.p2 = Process(target=self.com_socket, args=('192', 2001, self.sensores))
                # arrancamos los procesos
                p.start()
                print(self.sensores)
                self.ini = False
            else:
            '''

        print('sensores =', self.sensores)


        #self.update_battery_node()

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    

    def com_socket(self, ip, puerto, dic):
        # Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        server_address = (ip, puerto)
        print('connecting to {} port {}'.format(*server_address))
        sock.connect(server_address)
        try:
            while True:
                sensor_sin = sock.recv(1024).decode().split(":")
                self.include_dic(dic, sensor_sin)
        finally:
            print('closing socket')
            sock.close()

    def include_dic(self, dic, add):
        for sen in add:
            sensor = sen.split(";")
            if sensor[0] != "":
                # print('received =', sensor)
                dic[sensor[0]] = sensor[1::1]

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getAllSensorDistances method from Ultrasound interface
    #
    def Ultrasound_getAllSensorDistances(self):
        ret = ifaces.RoboCompUltrasound.SensorsState()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getAllSensorParams method from Ultrasound interface
    #
    def Ultrasound_getAllSensorParams(self):
        ret = ifaces.RoboCompUltrasound.SensorParamsList()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getAllSonarPose method from Ultrasound interface
    #
    def Ultrasound_getAllSonarPose(self):
        ret = ifaces.RoboCompUltrasound.SonarPoseList()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getBusParams method from Ultrasound interface
    #
    def Ultrasound_getBusParams(self):
        ret = ifaces.RoboCompUltrasound.BusParams()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getSensorDistance method from Ultrasound interface
    #
    def Ultrasound_getSensorDistance(self, sensor):
        ret = int()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getSensorParams method from Ultrasound interface
    #
    def Ultrasound_getSensorParams(self, sensor):
        ret = ifaces.RoboCompUltrasound.SensorParams()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getSonarsNumber method from Ultrasound interface
    #
    def Ultrasound_getSonarsNumber(self):
        ret = int()
        #
        # write your CODE here
        #
        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompUltrasound you can use this types:
    # RoboCompUltrasound.BusParams
    # RoboCompUltrasound.SensorParams
    # RoboCompUltrasound.SonarPose



    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):

        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
