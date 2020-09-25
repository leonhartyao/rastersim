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

#ifndef FDSTARPLANNER_H
#define FDSTARPLANNER_H

#include "abstractplanner.h"
#include <QImage>

class FocussedDStarPlanner: public AbstractPlanner {
	Q_OBJECT
public:
	FocussedDStarPlanner(QObject *parent = 0);
	~FocussedDStarPlanner();
	
	bool fullInit() const { return _fullInit; }
	void setFullInit(bool fullInit) { _fullInit = fullInit; }
	
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
		// TODO: merge List and blocked into one 32 bit word
		ListType list;
		bool blocked;
		int heapIndex;		
		Cell *pFocus;
		unsigned h_cost, k_cost;
		unsigned f_cost, fB_cost;
		
		inline bool operator<(const Cell &other) {
			if(fB_cost == other.fB_cost) {
				if(f_cost == other.f_cost) return k_cost < other.k_cost;
				else return f_cost < other.f_cost;
			} else return fB_cost < other.fB_cost;
		}
		inline bool operator>=(const Cell &other) { return ! (*this < other); }
	};
	inline unsigned dist(const Cell &c1, const Cell &c2) {
		unsigned dx = abs(c1.x - c2.x);
		unsigned dy = abs(c1.y - c2.y);
		unsigned dMin = qMin(dx, dy);
		unsigned dMax = qMax(dx, dy);
		return 7 * dMin + 5 * (dMax - dMin);
	}
	struct Cost {
		inline Cost(): c1(UINT_MAX), c2(UINT_MAX) { }
		inline Cost(unsigned c1, unsigned c2): c1(c1), c2(c2) { }
		inline bool operator<(const Cost &other) {
			if (c1 == other.c1) return c2 < other.c2;
			else return c1 < other.c1;
		}
		inline bool operator<=(const Cost &other) {
			if(c1 == other.c1) return c2 <= other.c2;
			else return c1 < other.c1;
		}		
		inline bool isNoVal() const { return (c1 == UINT_MAX && c2 == UINT_MAX); }
		unsigned c1, c2;
	};
	
	Cell *cells;	
	Cell **openHeap;
	unsigned openListLength;
	
	Cell *pRobot;
	unsigned d_curr;
	
	DebugLayer *listLayer;
	DebugLayer *backPtrLayer;
	QImage listMap;		
	void freeData();
	
	Cell *getMinState();
	Cost getMinVal();
	inline Cost getCost(const Cell &cell) { return Cost(cell.h_cost + dist(cell, *pRobot), cell.h_cost); }
	Cost processState(bool singleStep);
	void insert(Cell &cell, unsigned h_cost);
	void heapUp(Cell &cell);
	void heapDown(Cell &cell);
	void dumpCell(const Cell &cell);
	void dumpOpenHeap() const;
	void dumpOpenHeapLayer(unsigned index, unsigned level) const;	
	
	QAction *singleSteppingAction;
	QAction *singleStepAction;
	
	bool _fullInit;
	bool inhibitStep;	
};

#endif // FDSTARPLANNER_H
