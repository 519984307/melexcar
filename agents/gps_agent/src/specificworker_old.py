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
import numpy as np
import utm
import csv
from gps3 import gps3
from pyproj import Proj, transform, Transformer, Geod

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
        self.Period = 50

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 32
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)

        with open('GPS_new.csv', 'w+') as f:
            f.close()

        try:
            # signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            # signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            # signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            # signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            # signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            # signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
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
        self.num_gps = float(params["num_gps"])
        # self.theta = 0.27203221648765 - 0.02297200095 # + 3.1416  # Ajuste de orientación GPS a mapa DSR
        self.theta = float(params["theta"])
        self.phi = 0

        self.originX = float(params["originX"])
        self.originY = float(params["originY"])
        self.fixedX = float(params["fixedX"])
        self.fixedY = float(params["fixedY"])
        self.fixed_alpha = float(params["fixed_alpha"])
        self.N_average = int(params["N_average"])
        self.gps_dict = {}

        for i in range(int(self.num_gps)):
            name = params["name_" + str(i)]
            ip = params["ip_" + str(i)]
            port = params["port_" + str(i)]
            tx = float(params["tx_" + str(i)])
            ty = float(params["ty_" + str(i)])
            tz = float(params["tz_" + str(i)])

            # Diccionario [0]=ip [1]=port [2]=tx [3]=ty [4]=tz [5]=socket [6]=data_stream [7]=data  [8-1]=lat,long,alt,speed [13]=hdop
            self.gps_dict[name] = [ip, port, tx, ty, tz]

        for key in self.gps_dict:

            gps_socket = gps3.GPSDSocket()
            data_stream = gps3.DataStream()
            gps_socket.connect(host=self.gps_dict[key][0], port=self.gps_dict[key][1])
            print("CONEXION: ", self.gps_dict[key][0], ":", self.gps_dict[key][1])
            gps_socket.watch()

            self.gps_dict[key].append(gps_socket)
            self.gps_dict[key].append(data_stream)

            data_gps = {"lat": [None] * self.N_average,
                        "long": [None] * self.N_average,
                        "elev": [None] * self.N_average,
                        "speed": [None] * self.N_average,
                        "hdop": [None] * self.N_average
                        }

            self.gps_dict[key].append(data_gps)

            # inicio de valores lat,long,elev,speed, utmx, utmy, xgps e ygps
            for i in range(8):
                self.gps_dict[key].append(0.0)

        # Build rotation matrix
        rot = np.array([
            [np.cos(self.theta), -np.sin(self.theta), 0.0, 0.0],
            [np.sin(self.theta), np.cos(self.theta), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        )
        # Build shear/skew matrix
        m = np.tan(self.phi)
        skew = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [m, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        )
        # get affine transform
        self.a = rot @ skew
        print(self.a)
        # Build pipeline
        self.pt_transform = Transformer.from_pipeline(
            f"+proj=pipeline "
            # f"+step +proj=affine +xoff={origin_x} +yoff={origin_y} "
            f"+step +proj=affine +xoff={0} +yoff={0} "
            f"+s11={self.a[0, 0]} +s12={self.a[0, 1]} +s13={self.a[0, 2]} "
            f"+s21={self.a[1, 0]} +s22={self.a[1, 1]} +s23={self.a[1, 2]} "
            f"+s31={self.a[2, 0]} +s32={self.a[2, 1]} +s33={self.a[2, 2]} "
        )
        print(self.pt_transform)

        self.p1 = Proj("epsg:4326")
        self.p2 = Proj("epsg:23030")
        # self.fixedX = -371886.67816326916  #-574401.7908652775 -473268.6192685478
        # self.fixedY =4417967.8278194275 #4396225.790356246#4408260.03717906



        return True


    @QtCore.Slot()
    def compute(self):
        for key in self.gps_dict:
            try:
                self.gps_dict[key][5].next()
                # self.gps_dict[key].append(json.loads(self.gps_dict[key][5].response))
                # gps_json=json.loads(self.gps_dict[key][5].response)
                gps_node = self.g.get_node("gps")
                if gps_node:
                    gps_node.attrs['gps_connected'] = Attribute(True, self.agent_id)
                    self.g.update_node(gps_node)
            except:
                print("Error lectura:", key)
                gps_node = self.g.get_node("gps")
                if gps_node:
                    gps_node.attrs['gps_connected'] = Attribute(False, self.agent_id)
                    self.g.update_node(gps_node)


            # gps_json=self.gps_dict[key][8]
            self.gps_socket = self.gps_dict[key][5]
            self.data_stream = self.gps_dict[key][6]

            if (self.gps_socket.response):
                print("GPS_SOCKET_RESPONSE :", key)
                self.data_stream.unpack(self.gps_socket.response)

                if self.data_stream.TPV['lat'] != 'n/a':

                    self.gps_dict[key][7]["lat"].insert(0, float(self.data_stream.TPV['lat']))
                    del self.gps_dict[key][7]["lat"][-1]

                    self.gps_dict[key][7]["long"].insert(0, float(self.data_stream.TPV['lon']))
                    del self.gps_dict[key][7]["long"][-1]

                    self.gps_dict[key][7]["elev"].insert(0, float(self.data_stream.TPV['alt']))
                    del self.gps_dict[key][7]["elev"][-1]

                    self.gps_dict[key][7]["speed"].insert(0, float(self.data_stream.TPV['speed']))
                    del self.gps_dict[key][7]["speed"][-1]

                    self.gps_dict[key][8] = sum(self.gps_dict[key][7]["lat"]) / self.N_average
                    self.gps_dict[key][9] = sum(self.gps_dict[key][7]["long"]) / self.N_average
                    self.gps_dict[key][10] = sum(self.gps_dict[key][7]["elev"]) / self.N_average
                    self.gps_dict[key][11] = sum(self.gps_dict[key][7]["speed"]) * 0.2778 / self.N_average

                    # print("HDOP",self.data_stream.SKY['hdop'])

                    if self.data_stream.SKY['hdop'] != "n/a":
                        self.gps_dict[key][7]["hdop"] = self.data_stream.SKY['hdop'] * 4.75
                        # print(self.data_stream.SKY['hdop'],self.gps_dict[key][7]["hdop"])
                        # print("hdop: ",self.gps_dict[key][7]["hdop"])
                    # print(self.gps_dict[key][11])

                    u = utm.from_latlon(float(self.gps_dict[key][7]["lat"][0]), float(self.gps_dict[key][7]["long"][0]))
                    # u = utm.from_latlon(self.latitude, self.longitude)

                    self.xgps2 = u[0]
                    self.ygps2 = u[1]

                    self.UTMx = self.xgps2
                    self.UTMy = self.ygps2
                    # transform points into UTM

                    # self.UTMx = self.xgps2
                    # self.UTMy = self.ygps2
                    # self.xgps2, self.ygps2 = self.pt_transform.transform(self.xgps2, self.ygps2)

                    # self.xgps2=(self.xgps2*1000 - (self.fixedX*1000) + (self.originX))
                    # self.ygps2=(self.ygps2*1000 - (self.fixedY*1000) + (self.originY))

                    self.gps_dict[key][12] = self.xgps2
                    self.gps_dict[key][13] = self.ygps2

                    self.gps_dict[key][14], self.gps_dict[key][15] = self.pt_transform.transform(self.xgps2, self.ygps2)

                    self.gps_dict[key][14] = (self.gps_dict[key][14] * 1000 - (self.fixedX * 1000))
                    self.gps_dict[key][15] = (self.gps_dict[key][15] * 1000 - (self.fixedY * 1000))
                    print("X ",self.gps_dict[key][14]," ",self.gps_dict[key][15])

                    # print(key,"fixed x: ",self.gps_dict[key][12])
                    # print(key,"fixed y: ",self.gps_dict[key][13])

        if (self.num_gps > 1):
            geodesic = Geod(ellps='WGS84')
            self.azimut = geodesic.inv(self.gps_dict["gps_rear"][9], self.gps_dict["gps_rear"][8],
                                       self.gps_dict["gps_front"][9], self.gps_dict["gps_front"][8])

            self.fwd_azimut = self.azimut[0]
            self.alpha = self.fwd_azimut - self.theta * 180 / 3.14159265359

            # print("azimut: ",self.fwd_azimut)

        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.speed = 0
        self.UTMx = 0
        self.UTMy = 0
        self.mapx = 0
        self.mapy = 0
        self.hdop = 0.0

        for key in self.gps_dict:
            self.latitude = self.latitude + self.gps_dict[key][8] / self.num_gps
            self.longitude = self.longitude + self.gps_dict[key][9] / self.num_gps
            self.altitude = self.altitude + self.gps_dict[key][10] / self.num_gps
            self.speed = self.speed + self.gps_dict[key][11] / self.num_gps
            self.UTMx = self.UTMx + self.gps_dict[key][12] / self.num_gps
            self.UTMy = self.UTMx + self.gps_dict[key][13] / self.num_gps
            self.mapx = self.mapx + self.gps_dict[key][14] / self.num_gps
            self.mapy = self.mapy + self.gps_dict[key][15] / self.num_gps
            # if (float(self.gps_dict[key][7]["hdop"][0]>self.hdop):
            #    self.hdop = self.gps_dict[key][7]["hdop"]

        print("Mapx: ", self.mapx, "mm Mapy: ", self.mapy, " mm")
        print("alpha (º): ", self.alpha, "radianes =", self.alpha * 3.14159265359 / 180)
        print("speed: m/s", self.speed)
        print("HDOP = ", self.hdop)
        self.Update_gps_node()
        fa = open('GPS_new.csv', 'a')
        self.aaa = csv.writer(fa)
        if self.aaa is not None:
            self.aaa.writerow([self.latitude, self.longitude, self.altitude])

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def Update_gps_node(self):
        gps_node = self.g.get_node("gps")
        print(float(self.latitude))
        print(float(self.longitude))
        if gps_node:
            gps_node.attrs['gps_latitude'] = Attribute(float(self.latitude), self.agent_id )
            gps_node.attrs['gps_longitude'] = Attribute(float(self.longitude), self.agent_id )
            gps_node.attrs['gps_altitude'] = Attribute(float(self.altitude), self.agent_id )
            gps_node.attrs['gps_map_x'] = Attribute(float(self.mapx), self.agent_id )
            gps_node.attrs['gps_map_y'] = Attribute(float(self.mapy), self.agent_id )
            gps_node.attrs['gps_azimut'] = Attribute(float(self.fwd_azimut), self.agent_id )
            gps_node.attrs['gps_rot'] = Attribute(float(-self.alpha * np.pi/180), self.agent_id )
            gps_node.attrs['gps_UTMx'] = Attribute(float(self.UTMx), self.agent_id)
            gps_node.attrs['gps_UTMy'] = Attribute(float(self.UTMy), self.agent_id)
        self.g.update_node(gps_node)






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
