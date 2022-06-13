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

TO_MS = .2778
SCALAR_TO_M = 4.75


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100
        self.agent_id = 32
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)
        self.gps_dict = {}
        self.pos = 0

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
        self.num_gps = float(params["num_gps"])
        # self.theta = 0.27203221648765 - 0.02297200095 # + 3.1416  # Ajuste de orientación GPS a mapa DSR
        self.theta = float(params["theta"])
        self.phi = 0

        self.originX = float(params["originX"])
        self.cont_error = 0;
        self.originY = float(params["originY"])
        self.fixedX = float(params["fixedX"])
        self.fixedY = float(params["fixedY"])
        self.fixed_alpha = float(params["fixed_alpha"])
        self.N_average = int(params["N_average"])

        for i in range(int(self.num_gps)):
            name = params["name_" + str(i)]
            ip = params["ip_" + str(i)]
            port = params["port_" + str(i)]
            tx = float(params["tx_" + str(i)])
            ty = float(params["ty_" + str(i)])
            tz = float(params["tz_" + str(i)])

            gps_socket = gps3.GPSDSocket()
            data_stream = gps3.DataStream()
            gps_socket.connect(host=ip, port=port)
            print("CONEXION: ", ip, ":", port)
            gps_socket.watch()

            data_gps = {
                "lat":   np.zeros(self.N_average),
                "long":  np.zeros(self.N_average),
                "elev":  np.zeros(self.N_average),
                "speed": np.zeros(self.N_average),
                "hdop":  np.zeros(self.N_average),
            }

            self.gps_dict[name] = {
                "ip":     ip, #0
                "port":   port,
                "tx":     tx,
                "ty":     ty,
                "tz":     tz,
                "socket": gps_socket, #5
                "stream": data_stream,
                "data":   data_gps,
                "lat":   0.0,
                "long":   0.0,
                "elev":   0.0, #10
                "speed":   0.0,
                "utmX":   0.0,
                "utmX":   0.0,
                "mapX":   0.0,
                "mapY":   0.0, #15
            }

        # Build rotation matrix
        rot = np.array([
            [np.cos(self.theta), -np.sin(self.theta), 0.0, 0.0],
            [np.sin(self.theta),  np.cos(self.theta), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],])

        # Build shear/skew matrix
        m = np.tan(self.phi)
        skew = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [m, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],])

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
            f"+s31={self.a[2, 0]} +s32={self.a[2, 1]} +s33={self.a[2, 2]} ")
        print(self.pt_transform)
        return True

    @QtCore.Slot()
    def compute(self):
        for gps in self.gps_dict.values():
            try:
                gps["socket"].next()
                # self.gps_dict[key].append(json.loads(self.gps_dict[key][5].response))
                # gps_json=json.loads(self.gps_dict[key][5].response)
                if gps_node := self.g.get_node("gps"):
                    gps_node.attrs['gps_connected'] = Attribute(True, self.agent_id)
                    self.g.update_node(gps_node)
            except:
                print("Error lectura:", gps["name"])
                if gps_node := self.g.get_node("gps"):
                    gps_node.attrs['gps_connected'] = Attribute(False, self.agent_id)
                    self.g.update_node(gps_node)

            # gps_json=self.gps_dict[key][8]
            gps_socket = gps["socket"]
            data_stream = gps["stream"]
            print(data_stream)

            if (gps_socket.response):
                data_stream.unpack(gps_socket.response)

                if data_stream.TPV['lat'] != 'n/a':

                    gps["data"]["lat"][self.pos] = float(data_stream.TPV['lat'])
                    gps["data"]["long"][self.pos] = float(data_stream.TPV['lon'])
                    gps["data"]["elev"][self.pos] = float(data_stream.TPV['alt'])
                    gps["data"]["speed"][self.pos] = float(data_stream.TPV['speed'])

                    gps["lat"] = gps["data"]["lat"].mean()
                    gps["long"] = gps["data"]["long"].mean()
                    gps["elev"] = gps["data"]["elev"].mean()
                    gps["speed"] = gps["data"]["speed"].mean() * TO_MS

                    if data_stream.SKY['hdop'] != "n/a":
                        gps["data"]["hdop"] = data_stream.SKY['hdop'] * SCALAR_TO_M

                    #gps["utmX"], gps["utmY"] = utm.from_latlon(gps["data"]["lat"][0], gps["data"]["long"][0])
                    u= utm.from_latlon(gps["data"]["lat"][0], gps["data"]["long"][0])
                    gps["utmX"] = u[0]
                    gps["utmY"] = u[1]
                    gps["mapX"], gps["mapY"] = self.pt_transform.transform(gps["utmX"], gps["utmY"])

                    gps["mapX"] = gps["mapX"] * 1000 - (self.fixedX * 1000)
                    gps["mapY"] = gps["mapY"] * 1000 - (self.fixedY * 1000)

        self.pos = (self.pos + 1) % self.N_average

        if(self.num_gps > 1):
            geodesic = Geod(ellps='WGS84')
            self.azim = geodesic.inv(self.gps_dict["gps_rear"]["long"],  self.gps_dict["gps_rear"]["lat"],
                                     self.gps_dict["gps_front"]["long"], self.gps_dict["gps_front"]["lat"])
            self.alpha = self.azim[0] - np.degrees(self.theta)

        self.lat, self.long, self.alt, self.speed, self.utmx, self.utmy, self.mapx, self.mapy, self.hdop = 0, 0, 0, 0, 0, 0, 0, 0, 0
        for gps in self.gps_dict.values():
            self.lat += gps["lat"] / self.num_gps
            self.long += gps["long"] / self.num_gps
            self.alt += gps["elev"] / self.num_gps
            self.speed += gps["speed"] / self.num_gps
            self.utmx += gps["utmX"] / self.num_gps
            self.utmy += gps["utmY"] / self.num_gps
            self.mapx += gps["mapX"] / self.num_gps
            self.mapy += gps["mapY"] / self.num_gps
            self.hdop = self.hdop #if float(gps["data"]["hdop"][0]) < self.hdop else gps["data"]["hdop"][1]

        print("Mapx: ", self.mapx, "mm Mapy: ", self.mapy, " mm")
        print("alpha (º): ", self.alpha, "radianes =", np.radians(self.alpha))
        if self.alpha > 7:
            print("SE HA IDO")
            self.cont_error += 1
        print("speed: m/s", self.speed)
        print("HDOP = ", self.hdop)
        print("VECES_MAL: ",self.cont_error)
        self.Update_gps_node()

        file = open('GPS.csv', 'a')
        f_writer = csv.writer(file)
        if f_writer is not None:
            f_writer.writerow([self.lat, self.long, self.alt])
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def Update_gps_node(self):
        gps_node = self.g.get_node("gps")
        if gps_node:
            gps_node.attrs['gps_latitude'] = Attribute(float(self.lat), self.agent_id)
            print(self.lat)
            print(self.long)
            gps_node.attrs['gps_longitude'] = Attribute(float(self.long), self.agent_id)
            gps_node.attrs['gps_altitude'] = Attribute(float(self.alt), self.agent_id)
            gps_node.attrs['gps_map_x'] = Attribute(float(self.mapx), self.agent_id)
            gps_node.attrs['gps_map_y'] = Attribute(float(self.mapy), self.agent_id)
            gps_node.attrs['gps_azimut'] = Attribute(float(self.azim[0]), self.agent_id)
            gps_node.attrs['gps_rot'] = Attribute(float(np.radians(self.alpha)), self.agent_id)
            gps_node.attrs['gps_UTMx'] = Attribute(float(self.utmx), self.agent_id)
            gps_node.attrs['gps_UTMy'] = Attribute(float(self.utmy), self.agent_id)
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
