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
import math

from rich.console import Console
from genericworker import *

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

import numpy as np
from numpy.random import choice as CColor # ChooseColor
import os
import time
import operator
import multiprocessing
import cv2
from colored import fg
import json
# import trt_pose.coco
# import trt_pose.models
import copy
# import torch
# import torch2trt
# # from torch2trt import TRTModule
# import torchvision.transforms as transforms
import PIL.Image
# from trt_pose.parse_objects import ParseObjects
sys.path.append('/usr/local/lib/python3.6/pyrealsense2')
import pyrealsense2.pyrealsense2 as rs
# device = torch.device('cuda')
import math as m
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot
from pyproj import Proj, transform, Transformer, Geod

# Parametros
PMcR = [i for i in range(111, 231+1)] # Print Multicolor Range

# Colors
B = fg(250)
A = fg(14)
N = fg(208)
G = fg(244)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.IMPUT_WIDTH  = 848
        self.INPUT_HEIGHT = 480
        self.rotate = False
        self.fps_depth = 15;
        self.fps_color = 15;
        # Vars para luego
        self.profile = None
        self.depth_profile = None
        self.pipeline = None
        self.cam_map = {}
        self.filters = None
        self.serial_left = "108222250685"

        # Tiempos


        try:
            tm = TransformManager()
            rx = -0.610865
            ry = 0
            rz = 0
            tx = 0
            ty = 0
            tz = 0
            config = rs.config()
            config.enable_device(self.serial_left)
            config.enable_stream(rs.stream.color, self.IMPUT_WIDTH, self.INPUT_HEIGHT, rs.format.bgr8, 15)
            config.enable_stream(rs.stream.depth, self.IMPUT_WIDTH, self.INPUT_HEIGHT, rs.format.z16, 15)
            self.pipeline = rs.pipeline()
            self.pipeline.start(config)
            self.profile = self.pipeline.get_active_profile()
            self.depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
            tm.add_transform("cam", "origin",
                             pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([rx, ry, rz]),
                                                 [tx, ty, tz]))

            self.a = tm.get_transform("cam", "origin")


            self.pt_transform = Transformer.from_pipeline(
                f"+proj=pipeline "
                # f"+step +proj=affine +xoff={origin_x} +yoff={origin_y} "
                f"+step +proj=affine +xoff={0} +yoff={0} "
                f"+s11={self.a[0, 0]} +s12={self.a[0, 1]} +s13={self.a[0, 2]} "
                f"+s21={self.a[1, 0]} +s22={self.a[1, 1]} +s23={self.a[1, 2]} "
                f"+s31={self.a[2, 0]} +s32={self.a[2, 1]} +s33={self.a[2, 2]} "
            )


            self.cam_map[self.serial_left] = [self.pipeline, self.depth_profile, self.a, rs.frame, rs.points, rs.frame]


            # if self.rotate:
            #     self.IMPUT_WIDTH, self.INPUT_HEIGHT = self.INPUT_HEIGHT, self.IMPUT_WIDTH

        except:
            import tkinter as tk
            from tkinter import messagebox
            root = tk.Tk()
            root.withdraw()
            messagebox.showwarning('Atencion', '[!] La camara no esta conectada!')
            exit("[!] La camara no esta conectada!")

        #print(self.profile)
        # rs.options.set_option(rs.decimation_filter, rs.option.filter_magnitude, 2)
        # rs.option.filter_magnitude
        # rs.option.filter_option(rs.decimation_filter)
        # self.filters.append({"Decimate": rs.decimation_filter})
        # # print(self.depth_profile)
        # depth_intrinsics = self.depth_profile.get_intrinsics()
        # # print(depth_intrinsics)
        #
        # depth_to_color_extrin = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(
        #                                                                                                                                                                                                                                                                                                                  self.profile.get_stream(rs.stream.color))
        # color_to_depth_extrin = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(
        #     self.profile.get_stream(rs.stream.depth))

    def __del__(self):
        """Destructor"""
    def setParams(self, params):
        return True


    # def rotation_matrix(self, theta, alpha, ro ):
    #     Rx = np.matrix([[ 1, 0           , 0           ],
    #                [ 0, m.cos(theta),-m.sin(theta)],
    #                [ 0, m.sin(theta), m.cos(theta)]])
    #     Ry = np.matrix([[ m.cos(alpha), 0, m.sin(alpha)],
    #                [ 0           , 1, 0           ],
    #                [-m.sin(alpha), 0, m.cos(alpha)]])
    #
    #     Rz = np.matrix([[ m.cos(ro), -m.sin(ro), 0 ],
    #                [ m.sin(ro), m.cos(ro) , 0 ],
    #                [ 0           , 0            , 1 ]])
    #
    #     return Rz * Ry * Rx
    def compute(self):
        


        # Inicializamos la red y los elementos necesarios para mas adelante

        while True:
            self.peoplelist = self.computePilar()


        return True

    def startup_check(self):
        pass


    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getImage method from GiraffJetson interface
    #

        


    #
    # IMPLEMENTATION of getSkeleton method from GiraffJetson interface
    #



    # Metodos de CameraRGBD_Simple
    def CameraRGBDSimple_getAll(self, camera):
        
        return None
    #
    # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getDepth(self, camera):
        ret = RoboCompCameraRGBDSimple.TDepth()
        #
        # write your CODE here
        #
        return self.depth_img.data
    #
    # IMPLEMENTATION of getImage method from CameraRGBDSimple interface
    #
    def read_and_filter(self):
        for cam in self.cam_map:
            data = self.cam_map[cam][0].wait_for_frames()
            self.cam_map[cam][3] = data.get_depth_frame()
            self.cam_map[cam][5] = data.get_color_frame()
            pc = rs.pointcloud()
            pc.map_to(self.cam_map[cam][5])
            self.cam_map[cam][4] = pc.calculate(self.cam_map[cam][3])



    def Compute_Laser(self):
        ldata_write = []
        for cam in self.cam_map:
            if (self.cam_map[cam][4].size() == 0):
                continue
            tic = time.perf_counter()
            vertices = np.array(self.cam_map[cam][4].get_vertices(dims=2))
            vertices = np.c_[vertices, np.ones(len(vertices))]
            # FLOOR_DISTANCE_MINUS_OFFSET = self.cam_map[cam][2].translation().y() * 0.9
            vertices = vertices[vertices[:, 2] > 0.0]
            vertices = vertices @ self.cam_map[cam][2]
            vertices = vertices[abs(vertices[:, 1]) < 1.0] #[:,1] yv < max_down_height
            # vertices = vertices[vertices[:, 1] > -1.0] #[:,1] yv > max_up_height
            ang_rad = -np.arctan2(vertices[:, 0], vertices[:, 2])
            ang_index = (87 / (87*np.pi/180)) * ang_rad + (87/2.0)
            ang_index = ang_index.astype(int)

            vertices_2 = vertices[:] * vertices[:]
            vertices_2 = np.sum(vertices_2, axis=1) - 1
            # hor_bins = dict(zip(ang_index, vertices))
            vertices = np.c_[ang_index, vertices_2]
            vertices = vertices[abs(vertices[:, 0] - 43) <= 43] #[0 <= vertices[:, 0] <= 87]
            # vertices = vertices[vertices[:, 0] <= 87]
            vertices = np.sort(vertices, axis=0)[::-1]
            #
            # ang =
            #
            # dist = vertices[:, 1]

            hor_bins = dict(zip((vertices[:, 0] - 87/2.0) * (np.pi / 87), np.sqrt(vertices[:, 1])*1000))
            toc = time.perf_counter()
            print(hor_bins)
            print(f"Time {toc - tic:0.4f} seconds")
        for k, v in hor_bins.items:
             ldata_write.append(RoboCompLaser.TData(np.radians(angle), dist))


            # verts = rs.vertex(vertices[0])
            # vertices = rs.vertex(vertices)
            #FLOOR_DISTANCE_MINUS_OFFSET = self.cam_map[cam][2].translation().y() * 0.9
            # for i in range(0, int(self.cam_map[cam][4].size())):
            #     if vertices[i][2] >= 0.0:
            #         point = np.asarray([vertices[i][0], vertices[i][1], vertices[i][2], 1])
            #         #to_point = self.cam_map[cam][2] * np.asarray([vertices[i][0], vertices[i][1], vertices[i][2], 1]).T
            #         to_point = np.dot(self.cam_map[cam][2], point)
            #         #to_point = self.cam_map[cam][2].transform(vertices[i][0], vertices[i][1], vertices[i][2])
            #         xv = to_point[0]
            #         yv = to_point[1]
            #         zv = to_point[2]
                    #METER PARÁMETROS POR CONFIG
                    # if (yv < 1.0) and (yv > -1.0):
                    #     hor_angle = -math.atan2(xv, zv)
                    #     angle_index = int ((87 /87*np.pi/180) * hor_angle + (87/2.0))
                    #     if (angle_index >= 87 or angle_index < 0):
                    #         pass

            # FLOOR_DISTANCE_MINUS_OFFSET = extrin.translation().y() * 0.9
            # for i in range[0, len(points)]:
            #     if (vertices[i].z)
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

        # ret = RoboCompCameraRGBDSimple.TImage(
        #     cameraID=0,
        #     width=self.skeleton_white.shape[0],
        #     height=self.skeleton_white.shape[1],
        #     depth=3,
        #     focalx=0,
        #     focaly=0,
        #     alivetime=0,
        #     image=self.skeleton_white.tobytes()
        # )

        
        self.t_end_compute = time.time()
        print("////////////// tiempo de get image: ", self.t_end_compute - self.t_init_compute)
        return ret




















    #########################################################################################
    #########################################################################################
    """                                     INITIALIZE                                    """
    #########################################################################################
    #########################################################################################






    def preprocess(self, image):
        global device
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(device)
        image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return image[None, ...]









    def cam_matrix_from_intrinsics(self, i):
        return np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])





    #########################################################################################
    #########################################################################################
    """                                       COMPUTE                                     """
    #########################################################################################
    #########################################################################################

    def show_image(self, frames):
        # # Rotamos
        # color_image = cv2.rotate(color_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        cv2.imshow("Left_Cam", color_image)
        cv2.waitKey(1)
        return color_image

    def computePilar(self):
        # frames = self.pipeline.wait_for_frames()
        # color_image = self.show_image(frames)
        self.read_and_filter()
        self.Compute_Laser()

        # depth_frame = frames.get_depth_frame()
        # depth_data = np.asanyarray(depth_frame.get_data())
        # Rotamos
        # depth_data = cv2.rotate(depth_data, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # self.skeleton_image = np.zeros([480,640,1],dtype=np.float32)
        

        # self.depth_img = copy.deepcopy(depth_data)
        # depth_sensor = self.profile.get_device().first_depth_sensor()
        # depth_scale = depth_sensor.get_depth_scale()
        # depth_intrin = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        # color_intrin = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        # depth_to_color_extrin =  self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(self.profile.get_stream(rs.stream.color))
        # color_to_depth_extrin =  self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(self.profile.get_stream(rs.stream.depth))

        #self.t2 = time.time()
        





            

 


        def c2d(p):
            x,y = p
            depth_min = 0.1
            depth_max = 10.
            depth_point = rs.rs2_project_color_pixel_to_depth_pixel(depth_frame.get_data(), depth_scale, depth_min, depth_max, depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, [x,y])
            depth_point[0] = int(depth_point[0]+0.5)
            depth_point[1] = int(depth_point[1]+0.5)
            return depth_point

        def d2xyz(depth_point):
            depth_min = 0.01
            depth_max = 2.
            if np.any(depth_point == None):
                return False, [0,0,0]
            try:
                ret = True, rs.rs2_deproject_pixel_to_point(depth_intrin, depth_point, depth_scale*depth_data[depth_point[1], depth_point[0]])
                return ret
            except IndexError:
                return False, [0,0,0]




        


        return True
