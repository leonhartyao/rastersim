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

#ifndef DSTARPLANNER_H
#define DSTARPLANNER_H

#include "abstractplanner.h"
#include <QSize>
#include <QImage>


class DStarPlanner: public AbstractPlanner {
	Q_OBJECT
public:
	DStarPlanner(QObject *parent = 0);
	~DStarPlanner();
	
protected:
	void initMap(const QImage &map, const QRect &updateRegion = QRect());
	void calculatePath(InputUpdates updates);

	void drawDebugLayer(QPainter &painter, const DebugLayer *layer, const QRect &visibleArea, qreal zoomFactor);


private slots:
	void doSingleStep();
	void singleSteppingToggled(bool);

private:
	void doCalculatePath(InputUpdates updates, bool singleStep = false);
	
	enum ListType {
		List_New,
		List_Open,
		List_Closed,
	};
	struct Cell {
		Cell *backPtr;
		unsigned short x, y;
		unsigned h_cost, k_cost;
		bool blocked;
		int heapIndex;		
		ListType list;
	};
	Cell *cells;	
	Cell **openHeap;
	unsigned openListLength;
	
	void freeData();
	unsigned processState(bool singleStep);
	unsigned getKMin() const;
	void insert(Cell *pCell, unsigned h_cost);
	void heapUp(Cell *pCell);
	void heapDown(Cell *pCell);
	void dumpCell(const Cell *pCell);
	void dumpOpenHeap() const;
	void dumpOpenHeapLayer(unsigned index, unsigned level) const;
	
	DebugLayer *listLayer;
	DebugLayer *backPtrLayer;
	QImage listMap;
	
	QAction *singleSteppingAction;
	QAction *singleStepAction;
	
	bool inhibitStep;

};

#endif // ASTARPLANNER_H
