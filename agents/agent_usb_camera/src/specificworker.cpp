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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("./"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    try
    {
    //Parametros por el config
    pars.device  = params.at("device").value;
    pars.display = params.at("display").value == "true" or (params.at("display").value == "True");
    pars.compressed = params.at("compressed").value == "true" or (params.at("compressed").value == "True");
    std::cout << "Params: device" << pars.device << " display " << pars.display << " compressed: " << pars.compressed << std::endl;
    }
    catch(const std::exception &e)
    { std::cout << e.what() << " Error reading config params" << std::endl;};

	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);

	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = 40;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		{
		    current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        //open camera
        if( auto success = capture.open(pars.device); success != true)
        {
            qWarning() << __FUNCTION__ << " No camera found";
            std::terminate();
        }

        //compression params
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(50);

        this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    my_mutex.lock();
        capture >> frame;
//        if(pars.compressed)
//        {
//            cv::imencode(".jpg", frame, buffer, compression_params);
//            insert_camera_node_compressed();
//            update_usb_camera_compressed(wide_angle_camera_compressed_name);
//            //fps.print("Compression: " + std::to_string(frame.total() * frame.elemSize()/buffer.size()));
//            test_compressed();
//        }
//        else
//            qInfo() << __FUNCTION__ << "Compressed is not activated";

    my_mutex.unlock();

    if(pars.display)
    {
        if(int(frame.size().height)>0) {
            cv::imshow("Image", frame);
            cv::waitKey(2); // waits to display frame
        }
    }
    else
        qInfo() << __FUNCTION__ << "Display is not activated";


    insert_camera_node();
    cv::imencode(".jpg", frame, buffer, compression_params);
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    update_usb_camera(wide_angle_camera_name,frame, buffer );
}

void SpecificWorker::insert_camera_node()
{
    if (auto cam_node = G->get_node(wide_angle_camera_name); cam_node.has_value())
        cam_api = G->get_camera_api(cam_node.value());
    else {
        std::cout << "Controller-DSR terminate: could not find a camera node named "
                  << wide_angle_camera_name << std::endl;
        std::terminate();
    }
}

void SpecificWorker::update_usb_camera(std::string camera_name, const cv::Mat &v_image, const vector<uchar> compressed_data)
{
    if(auto node = G->get_node(camera_name); node.has_value())
    {
        std::vector<uint8_t> rgb; rgb.assign(v_image.data, v_image.data + v_image.total()*v_image.channels());
        G->add_or_modify_attrib_local<cam_rgb_att>(node.value(), rgb);
        G->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), v_image.cols);
        G->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), v_image.rows);
        G->add_or_modify_attrib_local<cam_rgb_depth_att>(node.value(), v_image.depth());
        G->add_or_modify_attrib_local<cam_rgb_cameraID_att>(node.value(),1);
        G->add_or_modify_attrib_local<compressed_data_att>(node.value(), compressed_data);
        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "Node wide_angle_camera not found";
}


void SpecificWorker::insert_camera_node_compressed()
{
    if (auto cam_node = G->get_node(wide_angle_camera_compressed_name); cam_node.has_value())
        cam_api = G->get_camera_api(cam_node.value());
    else {
        std::cout << "Controller-DSR terminate: could not find a camera node named "
                  << wide_angle_camera_compressed_name << std::endl;
        std::terminate();
    }
}

void SpecificWorker::update_usb_camera_compressed(std::string camera_name, const vector<uchar> compressed_data)
{
    if(auto node = G->get_node(camera_name); node.has_value())
    {
        G->add_or_modify_attrib_local<compressed_data_att>(node.value(), compressed_data);

        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "Node wide_angle_camera_compressed not found";
}

void SpecificWorker::test_compressed()
{
    if(auto node = G->get_node(wide_angle_camera_compressed_name); node.has_value())
    {
        vector<uchar> v = G->get_attrib_by_name<compressed_data_att>(node.value()).value();//nuestro buffer

        typedef struct TImage
        {
            bool compressed;
            int width;
            int height;
            int depth;
            vector<uchar> image;
        };

        TImage res;

        res.depth = frame.channels();
        res.height = frame.rows;
        res.width = frame.cols;
        res.compressed = pars.compressed;
        if(res.compressed)
            res.image = v;
        else
            res.image.assign(frame.data, frame.data + (frame.total() * frame.elemSize()));

        auto image = res;
        if (image.width !=0 and image.height !=0) {
            //qInfo() << image.width << "x" << image.height << " Size: " << image.image.size();
            if (image.compressed) {
                cv::Mat frameCompr = cv::imdecode(image.image, -1);
                cv::imshow("RGB image compressed", frameCompr);
            } else {
                //cv::Mat frame(cv::Size(image.width, image.height), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);
                cv::imshow("RGB image", frame);
            }
            cv::waitKey(1);
        }
        else
            qInfo() << "I dont have image";

    }
}

////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
////////////////////////////////////





