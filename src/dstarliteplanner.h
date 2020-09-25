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

#ifndef DSTARLITEPLANNER_H
#define DSTARLITEPLANNER_H

#include "abstractplanner.h"
#include <vector>
#include <cstdio>
#include <QImage>
class QAction;
class QActionGroup;

class DStarLitePlanner: public AbstractPlanner {
	Q_OBJECT
public:
	DStarLitePlanner(QObject *parent = 0);
	~DStarLitePlanner();

	void saveState(const QString &filename) const;
	void loadState(const QString &filename);
	void loadMapFromState(const QString &filename);
	
protected:
	void initMap(const QImage &map, const QRect &updateRegion = QRect());
	void calculatePath(InputUpdates updates);

	void drawDebugLayer(QPainter &painter, const DebugLayer *layer, const QRect &visibleArea, qreal zoomFactor);
	QString cellDetails(const QPoint &pos);

private slots:
	void doSteps(int max);
	void singleSteppingToggled(bool);

	void loadState();
	void loadMapState();	
		
private:
	
	// core D* Lite runtime data
	struct Key {
		unsigned k1, k2;
		Key(unsigned k1, unsigned k2): k1(k1), k2(k2) { }
		Key(): k1(0), k2(0) { }
		
		inline bool operator<(const Key &other) {
			if(k1 == other.k1) return k2 < other.k2;
			else return k1 < other.k1;
		}
		inline bool operator>=(const Key &other) { return !(*this < other); }		
	};
	struct Cell {
		unsigned short x, y;
		char neighborhoodIndex;
		char blocked;
		unsigned heapIndex;		
		unsigned g_cost, rhs;
		Key key; // cached key to speed up heap operations
		inline Key calculateKey(const Cell &start, unsigned k_m = 0) {
			unsigned k2 = g_cost < rhs ? g_cost : rhs;
			return Key(k2 + h_cost(*this, start) + k_m, k2);
		}
	};

	Cell *cells;	
	Cell *pGoal, *pStart, *pRobot;
	unsigned k_m;

	enum EdgeFlags {
		XMinEdge = 0x1,
		XMaxEdge = 0x2,
		YMinEdge = 0x4,
		YMaxEdge = 0x8
	};
	struct NeighborSpec {
		int ptrOffset;
		unsigned baseCost;
		NeighborSpec(unsigned ptrOffset, unsigned baseCost): ptrOffset(ptrOffset), baseCost(baseCost) { }
		NeighborSpec(): ptrOffset(0), baseCost(0) { }
	};
	typedef std::vector<NeighborSpec> Neighborhood;
	std::vector<Neighborhood> neighborhoods;
	
	// D* Lite core functions
	void doCalculatePath(InputUpdates updates, unsigned maxSteps = 0);
	bool computeShortestPath(unsigned maxSteps = 0); // returns true if plan is complete
	void updateVertex(Cell *c);
	static inline unsigned h_cost(const Cell &c1, const Cell &c2) {
		unsigned dx = abs(c1.x - c2.x);
		unsigned dy = abs(c1.y - c2.y);
		unsigned dMin = qMin(dx, dy);
		unsigned dMax = qMax(dx, dy);
		return 7 * dMin + 5 * (dMax - dMin);
	}	
	void doDebugAndPathExtract(bool pathExtract);
	void freeData();

	// heap management and debugging
	Cell **openHeap;
	unsigned openListLength;

	void insert(Cell &cell);
	void remove(Cell &cell);
	void heapUp(Cell &cell);
	void heapDown(Cell &cell);
	void dumpHeap(unsigned mark = 0) const;
	void dumpHeapLayer(unsigned index, unsigned level, unsigned mark = 0) const;	
	void checkHeap() const;
	unsigned checkHeapLayer(unsigned index, Key key) const;
	
	// generic debugging
	DebugLayer *listLayer;
	DebugLayer *costLayer, *backPtrs;
	QImage listMap;
	
	// single stepping
	QAction *singleSteppingAction;
	QActionGroup *singleStepGroup;	
	bool inhibitStep;	
	
	// stuff for saving & loading state
	QAction *loadStateAction;
	QAction *loadMapAction;
	
	int saveStateCounter;
	QImage map() const;
};

#endif // DSTARLITEPLANNER_H
