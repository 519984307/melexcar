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
#ifndef RSSISTATUS_H
#define RSSISTATUS_H

// Ice includes
#include <Ice/Ice.h>
#include <RSSIStatus.h>

#include <config.h>
#include "genericworker.h"


class RSSIStatusI : public virtual RoboCompRSSIStatus::RSSIStatus
{
public:
	RSSIStatusI(GenericWorker *_worker);
	~RSSIStatusI();

	RoboCompRSSIStatus::TRSSI getRSSIState(const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
