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
import math
import os
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
from pyModbusTCP.client import ModbusClient
import serial
sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)
import json

from pydsr import *


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 25

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 5
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
        # Comunicación PLC
        self.hostname = params["hostname"]
        self.SERVER_HOST_ROVER = "192.168.50.200"  # direccion del autómata
        self.SERVER_PORT_ROVER = 502  # puerto para las comunicaciones modbus tcp

        self.ROVER = ModbusClient()

        # define modbus server host, port
        self.ROVER.host(self.SERVER_HOST_ROVER)
        self.ROVER.port(self.SERVER_PORT_ROVER)

        #Direcciones Registros PLC
        self.MAP_DIRECCION = 0  # La direccion va desde 0 -> 24000. La posicion dentral del volante es 12000.
        self.MAP_VELOCIDAD = 1  # La velocidad va desde 0- > 5300.
        self.MAP_MAX_VELOCIDAD = 2  # Máxima valocidad a la que irá el vehículo.
        self.MAP_CAMBIO = 3  # valor 0 punto muerto, valor 1 adelante, valor 2 marcha atras.
        self.MAP_FRENO = 4  # Valores comprendidos entre 0 -> 9000 valor 9000 maxima frenada.
        self.MAP_SEGURIDAD = 5  # Escribimos 0 se queremos parar el vehiculo en modo seguridad, 1 para quitar la parada de seguridad.

        #Limitadores velocidad
        # self.max_melex_adv = 2650
        # self.max_melex_brake = 9000

        self.max_melex_adv = float(params["max_melex_adv"])
        self.max_melex_brake = float(params["max_melex_brake"])



        # Posicion Central volante
        # self.wheel_center = 12000
        self.wheel_center = float(params["wheel_center"])

        #Valores Iniciales
        self.advance = 0

        self.rotation = 0

        self.brake = 0
        self.atras = 1

        while not self.ROVER.is_open():
            print("waiting to connect melex")
            self.ROVER.open()

        self.ROVER.write_single_register(self.MAP_CAMBIO, 1)
        self.ROVER.write_single_register(self.MAP_SEGURIDAD, 1)
        self.ROVER.write_single_register(self.MAP_FRENO, 0)

        return True


    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')
        #response = os.system("ping -c 1 " + self.hostname)
        # and then check the response...
        print("MARCHA", self.ROVER.read_holding_registers(3))
        response = 0
        if response == 0:
            print(self.ROVER.read_holding_registers(self.MAP_SEGURIDAD))
            if self.ROVER.read_holding_registers(self.MAP_SEGURIDAD) == [0]:
                self.ROVER.write_single_register(self.MAP_SEGURIDAD, 1)
                print(self.hostname, 'is up!')
            self.set_movement()
        else:
            print(self.hostname, 'down')
            self.ROVER.write_single_register(self.MAP_SEGURIDAD, 0)
            conex = False
        self.read_odometry()
        ####################################### PROBAR ###############################################
        if self.ROVER.read_holding_registers(8) == [0] and self.advance < 0:
            self.ROVER.write_single_register(self.MAP_CAMBIO, 2)
            self.atras = -1
        elif self.ROVER.read_holding_registers(8) == [0] and self.advance > 0:
            self.ROVER.write_single_register(self.MAP_CAMBIO, 1)
            self.atras = 1
        self.send_json()





        return True

    def read_odometry(self):
        direccion =  self.ROVER.read_holding_registers(0)
        velocidad_reg1 = self.ROVER.read_holding_registers(8)
        velocidad_reg2 = self.ROVER.read_holding_registers(9)
        rad_giro = math.radians(((direccion[0] - 12000) * 35) / 12000)
        odometry_node = self.g.get_node("odometry")
        if odometry_node:
            odometry_node.attrs['odometry_vel'] = Attribute(float(velocidad_reg1[0]), self.agent_id)
            odometry_node.attrs['odometry_steer'] = Attribute(float(rad_giro), self.agent_id)
        self.g.update_node(odometry_node)
    def send_json(self):
        robot = self.g.get_node('robot')
        bateria = self.g.get_node('battery')
        gps = self.g.get_node('gps')
        odometry = self.g.get_node("odometry")
        nombre = "melexcar"
        if robot:
            ocupado = robot.attrs['robot_occupied'].value
        else:
            ocupado= "error"
        if bateria:
            carga = bateria.attrs['battery_load'].value
        else:
            carga ="Error"
        if gps:
            latitud = gps.attrs["gps_latitude"].value
            longitud = gps.attrs["gps_longitude"].value
            UTMx = gps.attrs["gps_UTMx"].value
            UTMy = gps.attrs["gps_UTMy"].value
        else:
            latitud ="Error"
            longitud = "Error"
            UTMx = "Error"
            UTMy = "Error"
        if odometry:
            velocidad = odometry.attrs['odometry_vel'].value
        data =json.dumps( {'Melex1':[{"Ocupado":ocupado, "Velocidad": velocidad,"CargaBatería":carga, "Coordenadas":[{ "Latitud": latitud, "Longitud":longitud, "UTMx": UTMx, "UTMy": UTMy}] }] })
        with open('json_data.json', 'w') as outfile:
            outfile.write(data)
        outfile.close()


    def set_movement(self):
        robot = self.g.get_node('robot')
        if robot:
            self.advance = robot.attrs['robot_ref_adv_speed'].value
            self.rotation = robot.attrs['robot_ref_rot_speed'].value
            self.brake = robot.attrs['robot_ref_brake_speed'].value
        try:
            if not self.ROVER.is_open():
                if not self.ROVER.open():
                    print("unable to connect to " + self.SERVER_HOST_ROVER + ":" + str(self.SERVER_PORT_ROVER))

            ACELERADOR = (self.atras * self.advance * self.max_melex_adv) / 1.3
            FRENO = self.brake
            if self.ROVER.is_open():
                print("ACELERADOR", ACELERADOR)
                self.ROVER.write_single_register( self.MAP_VELOCIDAD, int(ACELERADOR))
            DIRECCION = -self.rotation * self.wheel_center  + self.wheel_center
            print("DIRECCION ", DIRECCION)

            # print("ROTACION",self.rotation)
            # print("DIRECCION", DIRECCION)
            if self.ROVER.is_open():
                self.ROVER.write_single_register(self.MAP_DIRECCION, int(DIRECCION))
            if self.ROVER.is_open():
                self.ROVER.write_single_register(self.MAP_FRENO, int(100))
        except KeyboardInterrupt:
            print("No conectado")

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)





    




    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        # console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')
        pass

    def update_node(self, id: int, type: str):
        #console.print(f"UPDATE NODE: {id} {type}", style='green')
        pass

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):

        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
