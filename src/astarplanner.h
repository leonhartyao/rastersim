/*
 * Copyright 2016 Martin Seeman, IfA, TU Dresden, Germany
 * Copyright 2016 Chao Yao, IfA, TU Dresden, Germany
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ASTARPLANNER_H
#define ASTARPLANNER_H

#include "abstractplanner.h"
#include <QImage>

//#define HIGHQUALITYPATHPLANNER

class AStarPlanner: public AbstractPlanner {
public:
	AStarPlanner(QObject *parent = 0);
	~AStarPlanner();
	
protected:
	void initMap(const QImage &map, const QRect &updateRegion = QRect());
	void calculatePath(InputUpdates updates);
	
	void drawDebugLayer(QPainter &painter, const DebugLayer *layer, const QRect &visibleArea, qreal zoomFactor);
	
private:
	enum ListType {
		List_None,
		List_Open,
		List_Closed,
		List_Unwalkable
	};
	
	struct RasterElement {
		RasterElement *parent;
		int g_cost, f_cost; // storing f_cost here rather than in an open-list-element structure should be a performance optimization
#ifdef HIGHQUALITYPATHPLANNER
		int h_cost;
#endif
		ListType list;
		int openListIndex; // must be an index because otherwise the binary heap to array mapping would not be applicable
		unsigned short x, y; // coordinates of the element (can be deduced from the pointer, of course, but this would be a rather time consuming calculation)
	};
	RasterElement *rasterElements;	
	RasterElement **openList;
	
	void freeMemory();
	
	DebugLayer *visitedLayer;
	QImage visitedMap;
};

#endif // ASTARPLANNER_H
