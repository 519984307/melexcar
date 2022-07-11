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
#include <ranges>
#include <cppitertools/enumerate.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
    QLoggingCategory::setFilterRules("*.debug=false\n");

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
        inner_eigen = G->get_inner_eigen_api();
        rt = G->get_rt_api();

        //Body parts name
        body_parts = {"nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle", "neck"};
        //dsr update signals
//		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
//		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
//		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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

}

void SpecificWorker::compute()
{
    try
    {
        auto people_data = humancamerabody_proxy->newPeopleData();
        if (people_data.peoplelist.size() > 0)
        {
            cont= 0;
            auto people = people_data.peoplelist.front().joints;
            for (auto &[key, value]:people)
                std::cout << "key: " << key <<"value x: "<< value.x<<" value_y " << value.y <<" value_z"<< value.z << std::endl;

            insert_in_graph(people_data);
        }
        else
        {
            cont ++;
            if (cont>20)
                delete_people();
        }
    }
    catch (const Ice::Exception& e){ std::cout << e.what() << "Error connecting to Jetson Detector."<<std::endl;}

}



void SpecificWorker::insert_in_graph(RoboCompHumanCameraBody::PeopleData data) {

    if (auto world_node = G->get_node(world_name); world_node.has_value()) {
        if (auto p_in_G = G->get_nodes_by_type(person_type_name); not p_in_G.empty())
        {
            auto &person_node = p_in_G.front();//Cogemos el primero siempre
            if (auto person_rt = inner_eigen->transform(camera_exterior_name,person_node.name()); person_rt.has_value()) {
                const Eigen::Vector2d person_rt_floor = person_rt.value().head(2);
                std::vector<Eigen::Vector2d> mean_people(data.peoplelist.size());
                for (const auto &[i, person]: data.peoplelist | iter::enumerate) {
                    Eigen::Vector2d mean_pos;
                    for (auto &[join, value]: person.joints)
                        mean_pos += Eigen::Vector2d(-value.x*1000, value.z*1000);
                    mean_pos /= static_cast<double>(person.joints.size());
                    mean_people[i] = mean_pos;
                }
                auto res = std::ranges::min_element(mean_people, [pf = person_rt_floor](auto a, auto b) {
                    return (a - pf).norm() < (b - pf).norm();
                });
                if ((*res - person_rt_floor).norm() < 1000) {
                    auto var = *res;
                    Eigen::Vector3d new_person_pose{float(var[0]), float(var[1]), 0.0};
                    if (auto edge = rt->get_edge_RT(world_node.value(),person_node.id()); edge.has_value())
                    {
                        auto pose_in_world = inner_eigen->transform(world_name, new_person_pose, camera_exterior_name).value();
                        std::vector<float> pose {float(pose_in_world(0,0)),float(pose_in_world(1,0)),float(pose_in_world(2,0))};
                        std::cout << new_person_pose[0]<<new_person_pose[1] << std::endl;
                        G-> add_or_modify_attrib_local<rt_translation_att>(edge.value(),pose);
                        G->insert_or_assign_edge(edge.value());

                    }
                    else
                        std::cout <<"No RT edge found" << std::endl;
                }
                else
                    std::cout <<"People already inserted" << std::endl;
            }
            else
            {
                qWarning() << __FUNCTION__ << "No transform found between camera and person node";
            }
        }
        else
        {
            float pos_x = rand() % (120 - (-100) + 1) + (-100);
            float pos_y = rand() % (-100 - (-370) + 1) + (-370);
            int id = person_name_idx;
            person_name_idx += 1;
            std::string person_id_str = std::to_string(id);
            std::string node_name = "person_" + person_id_str;
            DSR::Node new_node = DSR::Node::create<person_node_type>(node_name);
            G->add_or_modify_attrib_local<person_id_att>(new_node, id);
            G->add_or_modify_attrib_local<parent_att>(new_node, world_node.value().id());
            G->add_or_modify_attrib_local<pos_x_att>(new_node, pos_x);
            G->add_or_modify_attrib_local<pos_y_att>(new_node, pos_y);
            G->insert_node(new_node);
            auto person = data.peoplelist.front();
            Eigen::Vector3d mean_pos;
            for (auto &[join, value]: person.joints)
                mean_pos += Eigen::Vector3d(value.x, value.y, 0);
            mean_pos /= static_cast<double>(person.joints.size());
            auto pose_in_world = inner_eigen->transform(world_name, mean_pos, camera_exterior_name).value();
            std::vector<float> pose {float(pose_in_world(0,0)),float(pose_in_world(1,0)),float(pose_in_world(2,0))};
            rt->insert_or_assign_edge_RT(world_node.value(),new_node.id(), pose, std::vector<float>{0.0,0.0,0.0});
       }
    }
}

//RoboCompHumanCameraBody::TJoints SpecificWorker::get_transformed_joint_list(const RoboCompHumanCameraBody::TJoints &joints)
//{
//    vector<cv::Point3f> joint_points; joint_points.assign(18, zero_pos);
//    vector<cv::Point2i> joint_pixels; joint_pixels.assign(18, zero_pix);
////    Eigen::Vector3f trans_vect_1(0, -0.06, -0.12);
////    Eigen::Vector3f trans_vect_2(0, -0.04, -1.55);
////    Eigen::AngleAxisf z_axis_rotation_matrix (servo_position, Eigen::Vector3f::UnitZ());
//    // Opposite angle sense than Coppelia
//    Eigen::AngleAxisf x_axis_rotation_matrix (-0.414, Eigen::Vector3f::UnitX());
//    int joint_counter = 0;
//    for(const auto &[key, item] : joints)
//        if (item.x != 0 and item.y != 0 and item.z != 0 and item.i != 0 and item.j != 0 and not (std::ranges::count(body_parts, key)))
//        {
////            std::cout << "KEY: " << key << std::endl;
//            joint_counter++;
////            cv::Point2i point_pix(item.i,item.j);
//            joint_pixels[jointPreference[std::stoi( key )]] = point_pix;
//            cv::Point3f point_robot(item.x*1000, item.y*1000, item.z*1000);
//            Eigen::Vector3f joint_pos(point_robot.x, point_robot.y, point_robot.z);
//            joint_pos = x_axis_rotation_matrix * (z_axis_rotation_matrix * joint_pos + trans_vect_1) + trans_vect_2;
//            point_robot.x = joint_pos.x();
//            point_robot.y = joint_pos.y();
//            point_robot.z = joint_pos.z();
//
//            joint_points[jointPreference[std::stoi( key )]] = point_robot;
//        }
//    return std::make_tuple(joint_points, joint_pixels);
//}
////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::delete_people() {
    if (auto p_in_G = G->get_nodes_by_type(person_type_name); not p_in_G.empty())
    {
        auto person_node = p_in_G.front();
        G->delete_edge(G->get_node(world_name).value().id(), person_node.id(), "RT");
        G->delete_node( person_node.id());
    }
}


/**************************************/
// From the RoboCompHumanCameraBody you can call this methods:
// this->humancamerabody_proxy->newPeopleData(...)

/**************************************/
// From the RoboCompHumanCameraBody you can use this types:
// RoboCompHumanCameraBody::TImage
// RoboCompHumanCameraBody::TGroundTruth
// RoboCompHumanCameraBody::KeyPoint
// RoboCompHumanCameraBody::Person
// RoboCompHumanCameraBody::PeopleData

