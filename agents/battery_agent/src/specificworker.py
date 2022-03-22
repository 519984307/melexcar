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
        self.Period = 100

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
        self.battery_variables = {}
        try:
            self.uart_port = params["uart_port"]
            self.baudrate = int(params["baudrate"])
            self.timeout = float(params["timeout"])
            print(self.timeout)
            self.ser = serial
        except:
            print("Error reading config params")
        try:
            self.ser = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.timeout
            )
            serial_port = "Open"
            print("The port %s is available" % self.ser)
        except serial.serialutil.SerialException:
            serial.Serial(self.uart_port, self.baudrate).close()
            # self.ser.close()
            print("Serial Closed")
            self.ser.open()
            print("Serial Open Again")
        return True


    @QtCore.Slot()
    def compute(self):
        try:
            for i in self.ser:
                variable = i.decode(encoding="utf-8").split()
                self.battery_variables[variable[0]] = float(variable[1])

        except:
            self.ser.close()
            self.ser.open()

        self.update_battery_node()

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def update_battery_node(self):
        battery_node = self.g.get_node('battery')
        if battery_node:
            battery_node.attrs['battery_load'].value = float(self.battery_variables['SOC']/10)
            battery_node.attrs['battery_V'] = Attribute(float(self.battery_variables['V']/1000), self.agent_id)
            battery_node.attrs['battery_P'] = Attribute(float(self.battery_variables['P']/100), self.agent_id)
            battery_node.attrs['battery_A'] = Attribute(float(self.battery_variables['I'] / 1000), self.agent_id)
            battery_node.attrs['battery_CE'] = Attribute(float(self.battery_variables['CE']/1000), self.agent_id)
            battery_node.attrs['battery_TTG'] = Attribute(float(self.battery_variables['TTG']/100), self.agent_id)
        #self.update_node_att(id, ['battery_load', 'battery_V', 'battery_A', 'battery_P', 'battery_CE', 'battery_TTG'])

        self.g.update_node(battery_node)




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
