/*
 *    Copyright (C) 2022 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include <iostream>
#include <string>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cppitertools/enumerate.hpp>
#include <opencv2/opencv.hpp>
#include <fps/fps.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/range.hpp>
#include  "../../../etc/pioneer_world_names.h"
#include "definitions.h"
#include "curvaturalaser.h"
#include "cdata.h"
#include "circle.h"


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera);
	RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera);
	RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera);
	RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState);
	RoboCompLaser::LaserConfData Laser_getLaserConfData();
	RoboCompLaser::TLaserData Laser_getLaserData();


    RoboCompLaser::TLaserData ldata_return;

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
    std::unique_ptr<DSR::RT_API> rt;
    std::shared_ptr<DSR::CameraAPI> cam_api;


	//DSR params
	std::string agent_name;
	int agent_id;

    struct CONSTANTS
    {
        float RIG_ELEVATION_FROM_FLOOR = 1.0; // m
        int MAX_LASER_BINS = 180;  // 2 for each degrees
        float TOTAL_HOR_ANGLE = M_PI;  // D455 opens 87 x 58 degrees
        float max_up_height = 0;
        float max_down_height = 0;
        int width = 424; //848
        int height = 240; //480
    };
    CONSTANTS consts;

    using Camera_Map =  std::map<std::string,
            std::tuple<rs2::pipeline,
                    rs2_intrinsics,
                    Eigen::Transform<float, 3, Eigen::Affine>,
                    rs2::frame,
                    rs2::points,
                    rs2::frame>>;
    bool startup_check_flag;
    Camera_Map cam_map;
    //worker
    int indice_grupo;
    std::vector<double> array_pixels, array_points;			// Array para deteccion de esquinas (r,theta)
    std::vector<int> breakpoints;

    // Vectores de coordenadas (cartesianas y polares)
    std::vector<float> x_coords, y_coords, angles, dists;

    // Posición del robot
    double robot_x, robot_y;

    std::vector<float> distanbreak;
    std::vector<float> angulobreak;
    std::vector<float> distancorners;
    std::vector<float> angulocorners;
    std::vector<float> distancirculo;
    std::vector<float> angulocirculo;


    std::vector<double> array_corners, virtual_corners, segmento, matriz_virtual, matriz_cov_circulos;
    int cont_segment, cont_circle;
    int contseg;
    CIRCULO   circle;
    CIRCULO circulo_new(double *array_pixels,int inicio,int fin);
    std::vector<SEGMENTO> array_segment_local;
    std::vector<CIRCULO> array_circle_local;
    MAPA_LASER scan_mapa, scan_maparef;
    MAPA_LASER mapa;
    // camera
    std::string serial_center, serial_left, serial_right;
    bool display_rgb = false;
    bool display_depth = false;
    bool display_laser = false;
    bool compressed = false;
    bool view = true;
    int num_planos =4;
    vector<int> compression_params_image;
    vector<int> compression_params_depth;

    rs2::config cfg_center, cfg_left, cfg_right;
    rs2_intrinsics center_cam_intr, left_cam_intr, right_cam_intr, center_depth_intr, left_depth_intr, right_depth_intr;
    rs2::context ctx;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};

	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    //mosaic
    cv::Mat virtual_frame;

    // filters
    struct filter_options
    {
    public:
        std::string filter_name;                                   //Friendly name of the filter
        rs2::filter &filter;                                       //The filter in use
        std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not

        filter_options( const std::string name, rs2::filter &flt) : filter_name(name),
                                                                    filter(flt),
                                                                    is_enabled(true)
        {};
        filter_options(filter_options&& other) : filter_name(std::move(other.filter_name)),
                                                 filter(other.filter),
                                                 is_enabled(other.is_enabled.load())
        {};
        const std::array<rs2_option, 5> possible_filter_options =
                {
                        RS2_OPTION_FILTER_MAGNITUDE,
                        RS2_OPTION_FILTER_SMOOTH_ALPHA,
                        RS2_OPTION_MIN_DISTANCE,
                        RS2_OPTION_MAX_DISTANCE,
                        RS2_OPTION_FILTER_SMOOTH_DELTA
                };
    };
    std::vector<filter_options> filters;
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;
    rs2::hole_filling_filter holef_filter;
    rs2::pipeline_profile profile;
    rs2::pointcloud pointcloud;
    rs2_error** error;

    // laser
    RoboCompLaser::TLaserData ldata;
    std::vector<RoboCompLaser::TLaserData> compute_laser(const Camera_Map &cam_map_extended);
    //void draw_laser(const RoboCompLaser::TLaserData &ldata);

    // images
    std::tuple<cv::Mat> mosaic(const Camera_Map &cam_map);
    Camera_Map& read_and_filter(Camera_Map &cam_map);
    void print_camera_params(const std::string &serial, const rs2::pipeline_profile &profile);
    void show_depth_images(Camera_Map &cam_map);
    std::mutex my_mutex;
    FPSCounter fps;
    vector<int> compression_params;
    vector<uchar> buffer;

    //methods
    void insert_camera_node();
    void update_three_camera_node(string camera_name, const cv::Mat &v_image, const vector<uchar> compressed_data);
    void insert_camera_node_compressed();
    // void update_three_camera_compressed(string camera_name, const vector<uchar> compressed_data);
    void insert_laser_node();
    void update_laser_node(string laser_name, const RoboCompLaser::TLaserData &ldata);

    //worker
    void iniciar_mapa_local();
    bool find_breakPoints(int contador); //
    std::vector<bool> MapBreakPoints (); //
    void features ( const std::vector<RoboCompLaser::TData> & lData, MAPA_LASER * scan_mapa, MAPA_LASER * scan_maparef);
    double distance(double x1, double y1, double x2, double y2); //
    double rad2angle ( double angle );
    double angle2rad ( double angle );
    void incluir_circulo(MAPA_LASER *mapa, CIRCULO *array_circle_local, CIRCULO circle, int cont_circle, double *matriz);
    void segment_new(int inicio, int fin);
    void incluir_segmento(); //
    void detectar_EsquinaVirtual();
    void incluir_esquina_virtual(MAPA_LASER *mapa);
    void map_copy(MAPA_LASER *mapa, MAPA_LASER *ref);
    void matriz_covarianza_circulo(CIRCULO circle, double *matriz, int cont);
    void segmento_Kai(int inicio, int fin);
    void obtener_mapa(MAPA_LASER *mapa);
    void matriz_covarianza_virtual(SEGMENTO segmento_2,SEGMENTO segmento_1, std::vector<double> matriz, int cont);
    double dist_euclidean2D(double x1, double y1, double x2, double y2);
    void extremo_kai(double rho, double theta, double alfa, double r, double *x1, double *y1);







};

#endif
