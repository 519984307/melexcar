/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "specificmonitor.h"
/**
* \brief Default constructor
*/
SpecificMonitor::SpecificMonitor(GenericWorker *_worker,Ice::CommunicatorPtr _communicator):GenericMonitor(_worker, _communicator)
{
	ready = false;
}
/**
* \brief Default destructor
*/
SpecificMonitor::~SpecificMonitor()
{
	std::cout << "Destroying SpecificMonitor" << std::endl;
}

void SpecificMonitor::run()
{
	initialize();
	ready = true;
	forever
	{
		//rDebug("specific monitor run");
		this->sleep(period);
	}
}

/**
 * \brief Reads components parameters and checks set integrity before signaling the Worker thread to start running
 *   (1) Ice parameters
 *   (2) Local component parameters read at start
 *
 */
void SpecificMonitor::initialize()
{
	rInfo("Starting monitor ...");
	initialTime=QTime::currentTime();
	RoboCompCommonBehavior::ParameterList params;
	readPConfParams(params);
	readConfig(params);
	if(!sendParamsToWorker(params))
	{
		rError("Error reading config parameters. Exiting");
		killYourSelf();
	}
	state = RoboCompCommonBehavior::State::Running;
	emit initializeWorker(period);
}

bool SpecificMonitor::sendParamsToWorker(RoboCompCommonBehavior::ParameterList params)
{
	if(checkParams(params))
	{
		//Set params to worker
		if(worker->setParams(params)) 
			return true;
	}
	else
	{
		rError("Incorrect parameters");
	}
	return false;

}

///Local Component parameters read at start
///Reading parameters from config file or passed in command line, with Ice machinery
///We need to supply a list of accepted values to each call
void SpecificMonitor::readConfig(RoboCompCommonBehavior::ParameterList &params )
{
    RoboCompCommonBehavior::Parameter aux;
    //aux.editable = false;

    configGetString( "","serial_left", aux.value, "");
    params["serial_left"] = aux;

    configGetString( "","serial_right", aux.value, "");
    params["serial_right"] = aux;

    configGetString( "","serial_center", aux.value, "");
    params["serial_center"] = aux;

    configGetString( "","display_rgb", aux.value, "false");
    params["display_rgb"] = aux;

    configGetString( "","display_depth", aux.value, "false");
    params["display_depth"] = aux;

    configGetString( "","display_laser", aux.value, "false");
    params["display_laser"] = aux;

    configGetString( "","compressed", aux.value, "true");
    params["compressed"] = aux;

    configGetString( "","view", aux.value, "false");
    params["view"] = aux;

    configGetString( "","max_up_height", aux.value, "1");  // + meters
    params["max_up_height"] = aux;

    configGetString( "","max_down_height", aux.value, "1");  // + meters
    params["max_down_height"] = aux;

    configGetString( "","agent_name", aux.value,"");
	params["agent_name"] = aux;
	configGetString( "","agent_id", aux.value,"false");
	params["agent_id"] = aux;

	configGetString( "","tree_view", aux.value, "none");
	params["tree_view"] = aux;
	configGetString( "","graph_view", aux.value, "none");
	params["graph_view"] = aux;
	configGetString( "","2d_view", aux.value, "none");
	params["2d_view"] = aux;
	configGetString( "","3d_view", aux.value, "none");
	params["3d_view"] = aux;
}

//Check parameters and transform them to worker structure
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}
