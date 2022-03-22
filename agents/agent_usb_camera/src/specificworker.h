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
//#include <opencv4/opencv2/core.hpp>
//#include <opencv4/opencv2/highgui.hpp>
//#include <opencv4/opencv2/imgproc.hpp>
//#include <opencv4/opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fps/fps.h>
#include <innermodel/innermodel.h>
#include  "../../../etc/pioneer_world_names.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
    //std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
    std::unique_ptr<DSR::RT_API> rt;
    std::shared_ptr<DSR::CameraAPI> cam_api;

    //DSR params
	std::string agent_name;
	int agent_id;

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
	bool startup_check_flag;

    std::mutex my_mutex;

    cv::VideoCapture capture;
    cv::Mat frame;

    vector<int> compression_params;
    vector<uchar> buffer;

    FPSCounter fps;

    struct PARAMS
    {
        std::string device = "/dev/video0";
        bool display = true;
        bool compressed = true;
    };
    PARAMS pars;

    void insert_camera_node();
    void update_usb_camera(string camera_name, const cv::Mat &v_image, const vector<uchar> compressed_data);

    void insert_camera_node_compressed();
    void update_usb_camera_compressed(string camera_name, const vector<uchar>);
    void test_compressed();
};

#endif
