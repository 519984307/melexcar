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
    //serial_left = params["serial_left"].value;
    serial_left = params["serial_left"].value;
    display_depth = (params["display_depth"].value == "true") or (params["display_depth"].value == "True");
    display_rgb = (params["display_rgb"].value == "true") or (params["display_rgb"].value == "True");
    display_laser = (params["display_laser"].value == "true") or (params["display_laser"].value == "True");
    compressed = (params["compressed"].value == "true") or (params["compressed"].value == "True");
    view = (params["view"].value == "true") or (params["view"].value == "True");
    consts.max_up_height = std::stof(params.at("max_up_height").value);
    consts.max_down_height = std::stof(params.at("max_down_height").value);

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
    this->Period = period;
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

        this->Period = period;
        timer.start(Period);
    }

    compression_params_image.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params_image.push_back(15);
    compression_params_depth.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params_depth.push_back(15);

    int fps_depth = 15;
    int fps_color = 15;

    // left camera
    try
    {
        cfg_left.enable_device(serial_left);
        cfg_left.enable_stream(RS2_STREAM_DEPTH, consts.width, consts.height, RS2_FORMAT_Z16, fps_depth);
        cfg_left.enable_stream(RS2_STREAM_COLOR, consts.width, consts.height, RS2_FORMAT_BGR8, fps_color);
        rs2::pipeline left_pipe;
        rs2::pipeline_profile profile_left = left_pipe.start(cfg_left);
        left_depth_intr = left_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        Eigen::Translation<float, 3> left_tr(0.0, 0.0, 0.0);
        // Eigen::Translation<float, 3> left_tr(0.0963, 0., 0.0578); // dos cámaras?
        Eigen::Matrix3f left_m;
        left_m = Eigen::AngleAxisf(-0.610865, Eigen::Vector3f::UnitX())
                  * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY())  //60 degrees
                  * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Transform<float, 3, Eigen::Affine> left_depth_extrinsics;;
        left_depth_extrinsics = left_tr;
        left_depth_extrinsics.rotate(left_m);
        cam_map[serial_left] = std::make_tuple(left_pipe, left_depth_intr, left_depth_extrinsics,  rs2::frame(), rs2::points(), rs2::frame());
        print_camera_params(serial_left, profile_left);
        qInfo() << __FUNCTION__ << " left-camera started";
    }
    catch(const std::exception &e)
    { std::cout << e.what()<< " Serial number:" << serial_left << std::endl; std::terminate();}

    // Filters
    rs2_set_option(dec_filter, RS2_OPTION_FILTER_MAGNITUDE, 2, error);
    filters.emplace_back("Decimate", dec_filter);
    filters.emplace_back("Spatial", spat_filter);
    filters.emplace_back("Temporal", temp_filter);
    filters.emplace_back("HFilling", holef_filter);

    //compression params
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(50);

    this->Period = 50;
    if(this->startup_check_flag)
        this->startup_check();
    else
        timer.start(Period);
}

void SpecificWorker::compute()
{
    auto &cam_map_extended = read_and_filter(cam_map);   // USE OPTIONAL
    auto ldata_local = compute_laser(cam_map_extended);
    auto &&[v_f] = mosaic(cam_map_extended);

    my_mutex.lock();
        virtual_frame=v_f.clone();
    my_mutex.unlock();

    if(compressed)
    {
        cv::imencode(".jpg", virtual_frame, buffer, compression_params);
        //insert_camera_node();
        cv::cvtColor(virtual_frame, virtual_frame, cv::COLOR_BGR2RGB);
        update_left_camera_node(virtual_frame, buffer);
    }

    if(display_depth)
        show_depth_images(cam_map_extended);
    if(display_rgb)
    {
        cv::cvtColor(virtual_frame, virtual_frame, cv::COLOR_BGR2RGB);
        cv::imshow("Virtual", virtual_frame);
        cv::waitKey(1);
    }
    if (display_laser)
        draw_laser(ldata_local);

    ldata_return = ldata_local;
    fps.print("FPS: ");

//    insert_laser_node();
    update_laser_node(ldata_local);

}
SpecificWorker::Camera_Map& SpecificWorker::read_and_filter(Camera_Map &cam_map)
{
    for (auto &[key, value] : cam_map)
    {
        //if(key != serial_center and key != serial_right) continue;
        auto &[my_pipe, intr, extr, depth_frame, points, color_frame] = value;
        rs2::frameset data = my_pipe.wait_for_frames();
        depth_frame = data.get_depth_frame(); // Find and colorize the depth dat
        color_frame = data.get_color_frame(); // Find and colorize the depth dat

        for (auto &&filter : filters)
            depth_frame = filter.filter.process(depth_frame);

        // rgb_list[i] = data.get_color_frame(); // Find the color data
        rs2::pointcloud pointcloud;
        pointcloud.map_to(color_frame);
        points = pointcloud.calculate(depth_frame);
    }
    return cam_map;
}


////////////////// MOSAIC  WITHOUT PROCESSING //////////////////////////////////////////
std::tuple<cv::Mat> SpecificWorker::mosaic(const Camera_Map &cam_map)
{
    // virtual frame
    cv::Mat frame_virtual = cv::Mat::zeros(consts.width, consts.height, CV_8UC3);
    const auto &[pipeL, intrL, extrL, depth_frameL, pointsL, color_frameL] = cam_map.at(serial_left);
    cv::Mat imgLeft(consts.height, consts.width, CV_8UC3, (char *) color_frameL.get_data());
    return std::make_tuple(imgLeft);
}

///////////////////// DISPLAY ////////////////////////////////////77
void SpecificWorker::show_depth_images(Camera_Map &cam_map)
{
    std::map<std::string,cv::Mat> image_stack;
    int w=0, h=0;
    for (auto &[key, value] : cam_map)
    {
        auto &[pipe, intr, extr, depth_frame, points, color_frame] = value;
        if(depth_frame)
        {
            rs2::frame depth_color = depth_frame.apply_filter(color_map);
            w = depth_frame.as<rs2::video_frame>().get_width();
            h = depth_frame.as<rs2::video_frame>().get_height();
            cv::Mat frame_depth(cv::Size(w,h), CV_8UC3, (void*)depth_color.get_data(), cv::Mat::AUTO_STEP);
            //image_stack.emplace_back(cv::Mat(cv::Size(w,h), CV_8UC3, (void*)depth_color.get_data(), cv::Mat::AUTO_STEP));
            image_stack[key] = frame_depth.clone();
        }
    }
    if(not cam_map.empty())
    {
        cv::Mat frame_final(cv::Size(w * image_stack.size(), h), CV_8UC3);
        image_stack[serial_left].copyTo(frame_final(cv::Rect(0,0,w,h)));
        image_stack[serial_center].copyTo(frame_final(cv::Rect(w,0,w,h)));
        image_stack[serial_right].copyTo(frame_final(cv::Rect(2*w,0,w,h)));
        cv::imshow("Depth mosaic", frame_final);
        cv::waitKey(1);
    }
}

void SpecificWorker::print_camera_params(const std::string &serial, const rs2::pipeline_profile &profile)
{
    float center_fov[2]; // X, Y fov
    const auto &[pipe, intr, extr, depth_frame, points, color_frame] = cam_map.at(serial);
    rs2_fov(&intr, center_fov);
    std::cout << "Camera " << serial << " started" << std::endl;
    std::cout << "  width: " << center_depth_intr.width << std::endl;
    std::cout << "  height: " << center_depth_intr.height << std::endl;
    std::cout << "  image center x: " << center_depth_intr.ppx << std::endl;
    std::cout << "  image center y: " << center_depth_intr.ppy << std::endl;
    std::cout << "  focal x: " << center_depth_intr.fx << std::endl;
    std::cout << "  focal y: " << center_depth_intr.fy << std::endl;
    std::cout << "  horizontal angle: " << center_fov[0] << std::endl;
    std::cout << "  vertical angle: " << center_fov[1] << std::endl;
    std::cout << "  extrinsics: " << extr.matrix() << std::endl;
    for (auto p : profile.get_streams())
        std::cout << "  stream ID: " << p.unique_id() << " - Stream name: " << p.stream_name() << std::endl;
}

RoboCompLaser::TLaserData SpecificWorker::compute_laser(const Camera_Map &cam_map_extended)
{
    using Point = std::tuple<float, float, float>;
    auto cmp = [](Point a, Point b)
    {
        auto &[ax, ay, az] = a;
        auto &[bx, by, bz] = b;
        return (ax * ax + ay * ay + az * az) < (bx * bx + by * by + bz * bz);
    };
    std::vector<std::set<Point, decltype(cmp) >> hor_bins(consts.MAX_LASER_BINS);   // lambda to sort elements on insertion
    //cout << "ÑAAAAAAAAAAAAAAAAAAAA" << endl;
    for( const auto &[key, value] : cam_map_extended)
    {
        const auto &[pipe, intrin, extrin, depth_frame, points, color_frame] = value;
        if(points.size() == 0) continue;
        const rs2::vertex *vertices = points.get_vertices();
        float FLOOR_DISTANCE_MINUS_OFFSET =  extrin.translation().y() * 0.9;  // X+ axis points donwards since the camera is rotated
     //   cout << "/////////////// adios" << FLOOR_DISTANCE_MINUS_OFFSET<< endl;
        for (size_t i = 0; i < points.size(); i++)
        {
            if(vertices[i].z >= 0.6) {
            //if(vertices[i].z >= 0.3 ) {
                auto to_point = extrin * Eigen::Vector3f{vertices[i].x, vertices[i].y, vertices[i].z};
                const float &xv = to_point[0];
                const float &yv = to_point[1];
                const float &zv = to_point[2];
//                cout << "YV: " << yv << endl;
//                cout << "MAX UP HEIGHT: " << consts.max_up_height << endl;
//                cout << "MAX DOWN HEIGHT: " << consts.max_down_height << endl;
                if ( (yv < consts.max_down_height) and (yv > consts.max_up_height) )  // central band - positive x downwards
                {
               //     cout<<"ENTRAAAAAAAAAA"<< endl;
                    float hor_angle = -atan2(xv, zv);
                    int angle_index = (int) ((consts.MAX_LASER_BINS / consts.TOTAL_HOR_ANGLE) * hor_angle +
                                             (consts.MAX_LASER_BINS / 2.f));
                    cout<<angle_index<<endl;
                    if (angle_index >= consts.MAX_LASER_BINS or angle_index < 0) continue;
                    hor_bins[angle_index].emplace(std::make_tuple(xv, yv, zv));

                }
            }
        }

    }
    RoboCompLaser::TLaserData ldata(consts.MAX_LASER_BINS);
    uint i = 0;
    for (auto &bin : hor_bins)
    {
        // int posicion= hor_bins - i;

        ldata[(hor_bins.size()-1) - i].angle = (i - consts.MAX_LASER_BINS / 2.f) * ( consts.TOTAL_HOR_ANGLE / consts.MAX_LASER_BINS );
        if (bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            ldata[i].dist = sqrt(X * X + Y * Y + Z * Z)*1000;
            cout<< "DISTANCIA" << ldata[i].dist<<endl;
            cout<< "ANG" << ldata[i].angle<<endl;
//            cout << "//////// " << ldata[i].dist << endl;
        }
        else
        {
            if(i>0) ldata[i].dist = ldata[i - 1].dist;  // link to the adjacent
            else ldata[i].dist = 10000;
        }
        i++;
    }
    return ldata;
}


void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata)
{
    if(ldata.empty()) return;

    const int lado = 800;
    const int semilado = lado/2;
    const int yoffset = 600;
    cv::Mat laser_img(cv::Size(lado, lado), CV_8UC3);
    laser_img = cv::Scalar(255,255,255);
    float scale = 0.1;
    cv::circle(laser_img, cv::Point(semilado,yoffset), 10, cv::Scalar(200,100,0), cv::FILLED, 8,0);
    float x = ldata.front().dist * sin(ldata.front().angle) * scale + lado/2;
    float y = yoffset - ldata.front().dist * cos(ldata.front().angle) * scale;
    cv::line(laser_img, cv::Point{semilado,yoffset}, cv::Point(x,y), cv::Scalar(0,200,0));
    for(auto &&l : iter::sliding_window(ldata, 2))
    {
        int x1 = l[0].dist * sin(l[0].angle) * scale + semilado;
        int y1 = yoffset - l[0].dist * cos(l[0].angle) * scale;
        int x2 = l[1].dist * sin(l[1].angle) * scale + semilado;
        int y2 = yoffset - l[1].dist * cos(l[1].angle) * scale;
        cv::line(laser_img, cv::Point{x1,y1}, cv::Point(x2,y2), cv::Scalar(0,200,0));
    }
    x = ldata.back().dist * sin(ldata.back().angle) * scale + semilado;
    y = yoffset - ldata.back().dist * cos(ldata.back().angle) * scale;
    cv::line(laser_img, cv::Point(x,y), cv::Point(semilado,yoffset), cv::Scalar(0,200,0));

    cv::imshow("Laser", laser_img);
    cv::waitKey(2);
}
///////////////nodes graph///////////////////


void SpecificWorker::update_left_camera_node( const cv::Mat &v_image, const vector<uchar> compressed_data)
{
    if(auto node = G->get_node(left_camera_name); node.has_value())
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
        qWarning() << __FUNCTION__ << "Node left_camera not found";
}

void SpecificWorker::update_laser_node(const RoboCompLaser::TLaserData &ldata)
{
    if (auto cam_left_node = G->get_node(left_camera_name); cam_left_node.has_value()) {
        if (auto laser_node = G->get_node(laser_left_name); laser_node.has_value()) {

            std::vector<float> dists;
            std::transform(ldata.begin(), ldata.end(), std::back_inserter(dists), [](const auto &l) { return l.dist; });
            std::vector<float> angles;
            std::transform(ldata.begin(), ldata.end(), std::back_inserter(angles),
                           [](const auto &l) { return l.angle; });
            G->add_or_modify_attrib_local<laser_dists_att>(laser_node.value(), dists);
            G->add_or_modify_attrib_local<laser_angles_att>(laser_node.value(), angles);
            G->update_node(laser_node.value());
        } else
            qWarning() << __FUNCTION__ << "No laser node found";
    }
}


//////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
//////////////////////////////////

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
    RoboCompCameraRGBDSimple::TImage im;
    my_mutex.lock();
    im.height = virtual_frame.rows;
    im.width = virtual_frame.cols;
    if(im.height != 0 and im.width != 0){
        im.compressed = compressed;
        if(true){

            //im.focalx = cam_map[camera].rs2_intrinsics.fx;
            //im.focaly = cam_map[camera].rs2_intrinsics.fy;
            //if(im.compressed)
            if ( im.compressed){
                vector<uchar> buffer;
                cv::Mat frame(cv::Size(im.width, im.height), CV_8UC3, &virtual_frame.data[0], cv::Mat::AUTO_STEP);
                cv::imencode(".jpg", frame, buffer, compression_params_image);
                std::cout << "raw: " << frame.total() * frame.elemSize() << " compressed: " << buffer.size() << " Ratio:"
                          << frame.total() * frame.elemSize() / buffer.size() << std::endl;
                im.image.assign(buffer.begin(), buffer.end());
            }
            else
                im.image.assign(virtual_frame.data, virtual_frame.data + (virtual_frame.total() * virtual_frame.elemSize()));


        }
    }
    else
        qInfo() << "I dont have image";

    my_mutex.unlock();
    return im;
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
//implementCODE

}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
//implementCODE

}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
    std::lock_guard<std::mutex> lg(my_mutex);
    return ldata_return;
}
