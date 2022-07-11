#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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

from rich.console import Console
from genericworker import *
import argparse
import sys
sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

import numpy as np
import jetson.inference
import jetson.utils
import time
import cv2
sys.path.append('/usr/local/lib/python3.6/pyrealsense2')

import pyrealsense2 as rs
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot
import interfaces as ifaces

print("PASA")


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.IMPUT_WIDTH = 424
        self.INPUT_HEIGHT = 240
        tm = TransformManager()
        config = rs.config()
        self.filters = [rs.decimation_filter(), rs.spatial_filter(), rs.temporal_filter(), rs.hole_filling_filter()]

        self.cam_map = {
            "r": { "id": "049522250171", },
            "l": { "id": "105322252279", },
            "c": { "id": "114122250668", },
        }
        camMats = {
            "c": { "T": [0, 0, -0.02], "R": [0, -0.244346, 0], },
            "l": { "T": [0,  0.02, 0], "R": [-np.pi/3, 0.244346, 0], },
            "r": { "T": [0, -0.02, 0], "R": [np.pi/3, 0.244346, 0], },
        }
        for k,v in self.cam_map.items():
            try:
                print("CAM", k)
                config.enable_device(v["id"])
                config.enable_stream(rs.stream.color, self.IMPUT_WIDTH, self.INPUT_HEIGHT, rs.format.bgr8, 15)
                config.enable_stream(rs.stream.depth, self.IMPUT_WIDTH, self.INPUT_HEIGHT, rs.format.z16, 15)
                v["pipeline"] = rs.pipeline()
                v["pipeline"].start(config)
                v["profile"] = v["pipeline"].get_active_profile()
                v["depth_profile"] = rs.video_stream_profile(v["profile"].get_stream(rs.stream.depth))
                tm.add_transform("cam", "origin", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz(camMats[k]["R"]), camMats[k]["T"]))
                v["mat"] = tm.get_transform("cam", "origin")
                v["depthIntrin"] = v["profile"].get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
                v["colorIntrin"] = v["profile"].get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
                v["depth2colorExtrin"] = v["profile"].get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(v["profile"].get_stream(rs.stream.color)) 
                v["color2depthExtrin"] = v["profile"].get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(v["profile"].get_stream(rs.stream.depth))
                v["points"] = rs.points
                v["depthFrame"] = rs.frame
                v["colorFrame"] = rs.frame
                v["depthScale"] = v["profile"].get_device().first_depth_sensor().get_depth_scale()
                self.cam_map[k] = v
                print("Done")
            except:
                import tkinter as tk
                from tkinter import messagebox
                root = tk.Tk()
                root.withdraw()
                messagebox.showwarning('Atencion', '[!] La camara no esta conectada!')
                exit("[!] La camara no esta conectada!")

        # Vars para luego
        self.ldata_read, self.ldata_write = [], []
        self.peoplelist, self.skeleton_img = None, None
        
        # Tiempos
        print('AAAAA')
        parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

        parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
        parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
        parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
        parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
        parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

        is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

        try:
	        self.opt = parser.parse_known_args()[0]
        except:
	        print("")
	        parser.print_help()
	        sys.exit(0)
        self.opt.network="pednet"
        self.net = jetson.inference.detectNet(self.opt.network, sys.argv, self.opt.threshold)
        # self.output_l = jetson.utils.videoOutput('images/test/left.avi', argv=sys.argv+is_headless)
        # self.output_r = jetson.utils.videoOutput('images/test/right.avi', argv=sys.argv+is_headless)
        # self.output_c = jetson.utils.videoOutput('images/test/center.avi', argv=sys.argv+is_headless)
        
    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        return True

    def compute(self):
        while True:
            self.read_and_filter()
            self.Compute_Laser()
            v_f, left, right, center = self.mosaic()
            cv2.imshow("CONJUNTA", v_f)
            cv2.waitKey(1)
            images = {
                "l": jetson.utils.cudaFromNumpy(left, isBGR=False), 
                "c": jetson.utils.cudaFromNumpy(center, isBGR=False),
                "r": jetson.utils.cudaFromNumpy(right, isBGR=False),
            }
            detections = {}
            for k in images.keys():
                detections[k] = self.net.Detect(images[k], overlay=self.opt.overlay), 

            xyzPoints = []                            
            for k,detection in detections.items(): 
                if len(detection) > 0 and detection != ([],):
                    xyzPoints.append(self.xyDetection2xyzPoints(detection, self.cam_map[k]))
                else:
                    print("No detections")
            
            for detection in xyzPoints:
                for a in detection:
                    print("TYPE:", type(a), "VALUE:", a)
                    
            for k in images.keys():
                images[k] = jetson.utils.cudaToNumpy(images[k])

            v_f_ssd = cv2.hconcat([images["l"], images["c"]])
            v_f = cv2.hconcat(([v_f_ssd, images["r"]]))
            cv2.imshow("CONJUNTA_SSD", v_f)
            cv2.waitKey(1)
            self.net.PrintProfilerTimes()

    def xyDetection2xyzPoints(self, detection, camera):
        xyzPoints = []
        print("DETECTION", detection)
        for point in detection:
            print("POINT", point[0])
            ret, xyzPoint = self.d2xyz(self.c2d(point[0].Center, camera), camera)
            if ret:
                xyzPoints.append(xyzPoint)
        return xyzPoints
    
    def d2xyz(self, depth_point, cam):
        if np.any(depth_point == None):
            return False, [0,0,0]
        try:
            depth_data = np.asanyarray(cam["depthFrame"].get_data())
            return True, rs.rs2_deproject_pixel_to_point(cam["depthIntrin"], depth_point, cam["depthScale"]*depth_data[depth_point[1], depth_point[0]])
        except IndexError:
            return False, [0,0,0]

    def c2d(self, p, cam):
        x,y = p
        depth_min = 0.1
        depth_max = 10.
        depth_point = rs.rs2_project_color_pixel_to_depth_pixel(cam["depthFrame"].get_data(), cam["depthScale"], depth_min, depth_max, cam["depthIntrin"], cam["colorIntrin"], cam["depth2colorExtrin"], cam["color2depthExtrin"], [x,y])
        depth_point = [int(x + 0.5) for x in depth_point]
        return depth_point

    def startup_check(self):
        pass

    def read_and_filter(self):
        for cam in self.cam_map.values():
            data = cam["pipeline"].wait_for_frames()
            cam["depthFrame"] = data.get_depth_frame()
            for filter in self.filters:
                cam["depthFrame"] = filter.process(cam["depthFrame"])
            cam["colorFrame"] = data.get_color_frame()
            pc = rs.pointcloud()
            pc.map_to(cam["colorFrame"])
            cam["points"] = pc.calculate(cam["depthFrame"])

    def Compute_Laser(self):
        self.ldata_write = []
        i = 0
        tic = time.perf_counter()
        for cam in self.cam_map.values():
            if cam["points"].size() == 0:
                continue
            vertices = np.array(cam["points"].get_vertices(dims=2))
            vertices = np.c_[vertices, np.ones(len(vertices))]
            vertices = vertices[vertices[:, 2] > 0.6]
            vertices = (cam["mat"] @ vertices.T).T
            if i == 0:
                allvertices = vertices
                i = i+1
            else:
                allvertices = np.r_[allvertices, vertices]
        allvertices = allvertices[allvertices[:, 0] <= 0.2]
        allvertices = allvertices[allvertices[:, 0] >= 0]  #Corte con el suelo
        ang_rad = np.arctan2(allvertices[:, 1], allvertices[:, 2])
        ang_index = ((180 / (np.pi)) * ang_rad + (180/2.0)).astype(int)
        vertices_2 = np.sqrt( np.sum(allvertices[:] * allvertices[:], axis=1)-1)*1000
        vertices_2 = np.c_[ang_index, vertices_2]
        vertices_2 = vertices_2[vertices_2[:, 0] <= 180]
        vertices_2 = vertices_2[vertices_2[:, 0] >= 0]
        # self.vertices_2 = self.vertices_2[abs(self.vertices_2[:, 0] - 43) <= 43] #[0 <= vertices[:, 0] <= 87]
        vertices_2 = vertices_2[np.lexsort((vertices_2[:,1], vertices_2[:,0]))][:: -1]
        # vertices_2 = vertices_2[:: -1]
        hor_bins = dict(zip((vertices_2[:, 0] - (180/2.0)) * (np.pi/180),vertices_2[:, 1]))
        toc = time.perf_counter()
        print(f"Time {toc - tic:0.4f} seconds")
        for k, v in hor_bins.items():
            self.ldata_write.append(ifaces.RoboCompLaser.TData(k, v))
        self.ldata_read, self.ldata_write = self.ldata_write, self.ldata_read
        # print(self.ldata_read)

    def mosaic(self):
        winCenters = {}
        for char,cam in self.cam_map.items():
            img = np.asanyarray(cam["colorFrame"].get_data())
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            winCenters[char] = cv2.rotate(img, 0)

        v_f = cv2.hconcat([winCenters["l"], winCenters["c"]])
        v_f = cv2.hconcat(([v_f, winCenters["r"]]))

        return v_f, winCenters["l"], winCenters["r"], winCenters["c"]

    # CameraRGBD_Simple interface
    def CameraRGBDSimple_getAll(self, camera):
        
        return None
    def CameraRGBDSimple_getDepth(self, camera):
        ret = RoboCompCameraRGBDSimple.TDepth()
        return self.depth_img.data

    def CameraRGBDSimple_getImage(self, camera):
        """
        Devuelve la imagen rgb que ve la camara.
        """
        print("////////////// GETTING IMAGE //////////////")
        self.t_init_compute = time.time()
        ret = RoboCompCameraRGBDSimple.TImage(
            compressed=True,
            cameraID=0,
            width=self.skeleton_img.shape[0],
            height=self.skeleton_img.shape[1],
            depth=3,
            focalx=0,
            focaly=0,
            alivetime=0,
            image=self.skeleton_img.tobytes()
        )        
        self.t_end_compute = time.time()
        print("////////////// tiempo de get image: ", self.t_end_compute - self.t_init_compute)
        return ret

    # Laser interface
    def Laser_getLaserAndBStateData(self):
        ret = ifaces.RoboCompLaser.TLaserData()
        bState = ifaces.RoboCompGenericBase.TBaseState()
        return [ret, bState]

    def Laser_getLaserConfData(self):
        ret = ifaces.RoboCompLaser.LaserConfData()
        return ret

    def Laser_getLaserData(self):
        return self.ldata_read
