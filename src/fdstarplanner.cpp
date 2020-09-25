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

#include "fdstarplanner.h"
#include <new>
#include <cstdio>
#include <QPainter>
#include <QAction>

#define OBSTACLE_COST	2000000000U

FocussedDStarPlanner::FocussedDStarPlanner(QObject *parent):
	AbstractPlanner(parent),
	cells(NULL), openHeap(NULL), openListLength(0),
	listLayer(NULL), backPtrLayer(NULL),
	_fullInit(false), inhibitStep(false)
{
	singleSteppingAction = new QAction(tr("Single stepping"), this);
	singleSteppingAction->setCheckable(true);
	singleStepAction = new QAction(tr("Next Step"), this);
	connect(singleStepAction, SIGNAL(triggered(bool)), this, SLOT(doSingleStep()));
	connect(singleSteppingAction, SIGNAL(toggled(bool)), singleStepAction, SLOT(setEnabled(bool)));
	connect(singleSteppingAction, SIGNAL(toggled(bool)), this, SLOT(singleSteppingToggled(bool)));
	singleStepAction->setEnabled(singleSteppingAction->isChecked());
	
	addAction(singleSteppingAction);
	addAction(singleStepAction);	
}

FocussedDStarPlanner::~FocussedDStarPlanner() {
	freeData();
}
	
void FocussedDStarPlanner::freeData() {
	if(cells) {
		delete[] cells;
		cells = NULL;
	}
	if(openHeap) {
		delete[] openHeap;
		openHeap = NULL;
	}
}

void FocussedDStarPlanner::initMap(const QImage &map, const QRect &updateRegion) {	
	if(updateRegion.isNull()) {
		freeData();
		cells = new (std::nothrow) Cell[map.width() * map.height()];
		openHeap = new (std::nothrow) Cell *[map.width() * map.height() + 1];
		openListLength = 0;
		listMap = QImage();
		
		if(!cells || !openHeap) {
			printf("Failed allocating runtime memory\n");
			return;
		}		
		
		Cell *pCell = cells;
		for(int y = 0; y < mapHeight(); y++) {
			const unsigned char *pCost = (const unsigned char *)map.scanLine(y);
			for(int x = 0; x < mapWidth(); x++) {
				pCell->x = x;
				pCell->y = y;
				pCell->backPtr = NULL;
				pCell->pFocus = NULL;
				pCell->list = List_New;
				pCell->heapIndex = 0;
				pCell->h_cost = pCell->f_cost = pCell->fB_cost = 0;
				pCell->blocked = (*pCost++ > 0);
				pCell++;
			}			
		}	
	} else {
		// It's a map update: incorporate cost changes
		int w = mapWidth();
		int h = mapHeight();
		for(int y = updateRegion.top(); y <= updateRegion.bottom(); y++) {
			const unsigned char *pCost = (const unsigned char *)map.scanLine(y) + updateRegion.left();
			Cell *pCell = cells + w * y + updateRegion.left();
			for(int i = 0; i < updateRegion.width(); i++) {
				bool newBlocked = (*pCost++ > 0);
				if(newBlocked != pCell->blocked) {
					pCell->blocked = newBlocked;
					// add the changed cell itself to the OPEN list
					if(pCell->list == List_Closed) insert(*pCell, pCell->h_cost);
										
					if(!pCell->blocked) {
						// if a cell has been unblocked, add all neighbors to the open list.
						// Note: In the paper, only arc cost changes are mentioned, but we are working on cells, i.e. if
						// chaning a cell's cost this influences all arcs from this cell to its neighbors.
						for(int iy = pCell->y - 1; iy <= pCell->y + 1; iy++) {
							if((unsigned)iy >= (unsigned)h) continue;
							for(int ix = pCell->x - 1; ix <= pCell->x + 1; ix++) {
								if((unsigned)ix >= (unsigned)w) continue;
								Cell *pNeighbor = cells + iy * w + ix;
								if(pNeighbor->list == List_Closed) insert(*pNeighbor, pNeighbor->h_cost);
							}
						}
					}					
				}
				pCell++;			
			}		
		}
	}	
}

void FocussedDStarPlanner::singleSteppingToggled(bool enabled) {
	if(!enabled) {
		doCalculatePath(0, false);	
		emit dataChanged();
	}
}

void FocussedDStarPlanner::doSingleStep() {
	doCalculatePath(0, true);
	// Inform GUI for redrawing
	emit dataChanged();
}

void FocussedDStarPlanner::calculatePath(InputUpdates updates) {
	inhibitStep = singleSteppingAction->isChecked();			
	doCalculatePath(updates, inhibitStep);
}

void FocussedDStarPlanner::doCalculatePath(InputUpdates updates, bool singleStep) {
	
	if(!cells || !openHeap) {
		setError("Planner memory allocation error");
		return;
	}
	
	unsigned width = mapWidth();
	unsigned height = mapHeight();
	// prepare image that shows list membership (size & color table)
	if(listMap.size() != mapSize()) {
		listMap = QImage(mapSize(), QImage::Format_Indexed8);
		listMap.setColorTable(QVector<QRgb>() << qRgba(0, 0, 0, 0) << qRgba(0, 255, 255, 192) << qRgba(255, 255, 0, 192) << qRgba(0, 128, 255, 128) << qRgba(255, 200, 0, 128) << qRgb(0, 200, 0));
	}
	
	QPoint startPos = start().pos().toPoint();
	QPoint goalPos = goal().pos().toPoint();
	Cell *pStart = cells + (int)startPos.y() * width + (int)startPos.x();
	Cell *pGoal = cells + (int)goalPos.y() * width + (int)goalPos.x();

	// check validity of start & goal
	if(pStart->blocked) {
		setError("Start position blocked");
		return;
	} else if(pGoal->blocked) {
		setError("Goal position blocked");
		return;
	}

	// if reusing knowledge from previous calls is not possible, (re-)initialize planner state
	if(updates & ~(UpdatedStart | UpdatedMap)) {
		for(Cell *pCell = cells; pCell < (cells + width * height); pCell++) {
			pCell->list = List_New;
			pCell->backPtr = NULL;
			pCell->heapIndex = 0;
			pCell->h_cost = 0;
		}

		pRobot = pStart;
		d_curr = 0;
		
		openListLength = 1;
		pGoal->fB_cost = pGoal->f_cost = pGoal->h_cost = pGoal->k_cost = 0;
		pGoal->list = List_Open;
		pGoal->heapIndex = 1;
		openHeap[1] = pGoal;
	}
	
	bool success = true;
	if(inhibitStep) {
		inhibitStep = false;
		setError("Single stepping enabled...");
		success = false;
	} else {	
		if(updates & (NewMap | UpdatedGoal)) {
			while(true) {
				Cost val = processState(singleStep);
				if(!_fullInit && pStart->list == List_Closed) break;
				if(openListLength == 0) break;
				if(val.c2 >= OBSTACLE_COST) break; // no error handling here since pStart is checked for valid costs after the loop
				if(singleStep) {
					success = false;
					setError("Not yet ready...");
					break;
				}								
			}
			if(success && (pStart->list != List_Closed || pStart->h_cost >= OBSTACLE_COST)) {
				setError("No Path found");
				success = false;
			}
			
		} else {			
			if(pStart != pRobot) {
				d_curr += dist(*pStart, *pRobot) + 1;			
				pRobot = pStart;	
			}		

			Cost val = getMinVal();
			if(pStart->list == List_New || val < getCost(*pStart)) {
				while(openListLength > 0) {
					val = processState(singleStep);
					
					if(pStart->list != List_New && getCost(*pStart) <= val) break;
					
					if(val.c2 >= OBSTACLE_COST) {
						setError("No Path found");
						success = false;
						break;
					}
					if(singleStep) {
						success = false;
						setError("Not yet ready...");
						break;
					}						
				}
			}

			if(success && (pStart->list == List_New || pStart->h_cost >= OBSTACLE_COST)) {
				setError("No Path found");
				success = false;
			}
		}
	}
	
	// prepare debug layers
	const Cell *pCell = cells;
	for(unsigned y = 0; y < height; y++) {
		unsigned char *pMap = listMap.scanLine(y);
		for(unsigned x = 0; x < width; x++) {
			switch(pCell->list) {
			case List_Closed: *pMap++ = (pCell->pFocus == pRobot) ? 2 : 4; break;
			case List_Open: *pMap++ = (pCell->pFocus == pRobot) ? 1 : 3; break;
			default: *pMap++ = 0;
			}
			pCell++;
		}
	}
	if(openListLength >= 1) listMap.setPixel(openHeap[1]->x, openHeap[1]->y, 5);
	
	if(!listLayer) addDebugLayer(listLayer = new DebugLayer(tr("Lists (cyan = open, yellow = closed)")));
	if(!backPtrLayer) addDebugLayer(backPtrLayer = new DebugLayer(tr("Backpointers"), 0));
	Path p;
	
	if(success) {
		// follow the backpointers to construct the path
		Cell *pCell = pStart;
		
		int pathLength = 0;	
		while(true) {
			pathLength++;
			if(pCell->blocked) {
				// sanity check: path planned through blocked area
				setError("Path blocked");
				success = false;
				break;
			}
			if(pCell == pGoal) break;

			pCell = pCell->backPtr;
			if(!pCell) {
				// internal error: backpointers do not form a sequence
				setError("NULL pointer in backpointer sequence");
				success = false;
				break;
			}
			if(pathLength > 1000000) {
				// sanity check: probably loop in backpointer sequence
				setError("Path too long");
				success = false;
				break;
			}
		}
		if(success) {
			// path is valid -> store it
			printf("Path length = %d\n", pathLength);
			
			p.resize(pathLength);
			pathLength = 0;
			pCell = pStart;
			while(true) {		
				p[pathLength++] = QPointF(pCell->x, pCell->y);				
				if(pCell == pGoal) break;
				pCell = pCell->backPtr;
			}
		}
	}

	setPath(p);	
}

FocussedDStarPlanner::Cell *FocussedDStarPlanner::getMinState() {
	if(openListLength > 0) {
		while(true) {
			Cell *pMin = openHeap[1];
			if(pMin->pFocus == pRobot) return pMin;
			
			// correct f[B]_cost and reposition in the heap
			pMin->f_cost = pMin->k_cost + dist(*pMin, *pRobot);
			pMin->fB_cost = pMin->f_cost + d_curr;
			pMin->pFocus = pRobot;
			heapUp(*pMin);
			heapDown(*pMin);		
		}
	}
	return NULL;
}

FocussedDStarPlanner::Cost FocussedDStarPlanner::getMinVal() {
	Cell *pMin = getMinState();
	if(pMin) return Cost(pMin->f_cost, pMin->k_cost);
	else return Cost();
}

FocussedDStarPlanner::Cost FocussedDStarPlanner::processState(bool singleStep) {
	Cell *pMin = getMinState();
	// error if open list is empty
	if(!pMin) return Cost();
	
	// remove first entry from the open list
	pMin->list = List_Closed;
	pMin->heapIndex = 0;	
	if(--openListLength){
		openHeap[1] = openHeap[openListLength + 1]; //move the last item in the heap up to slot #1
		openHeap[1]->heapIndex = 1;
		heapDown(*openHeap[1]);
	}	
	Cost val(pMin->f_cost, pMin->k_cost);
	unsigned k_val = pMin->k_cost;
	if(singleStep) printf("### processState for (%d, %d), h_cost = %u, k_val = k_cost = %u ###\n", pMin->x, pMin->y, pMin->h_cost, k_val);	
	
	Cell *pNeighbors[8];
	unsigned c_cost[8];
	unsigned numNeighbors = 0;
	for(int y = pMin->y - 1; y <= pMin->y + 1; y++) {
		for(int x = pMin->x - 1; x <= pMin->x + 1; x++) {
			if(y == pMin->y && x == pMin->x) continue;
			if((unsigned)y >= (unsigned)mapHeight() || (unsigned)x >= (unsigned)mapWidth()) continue;			
			Cell *pNeighbor = cells + y * mapWidth() + x;
			pNeighbors[numNeighbors] = pNeighbor;
			if(pNeighbor->blocked || pMin->blocked) c_cost[numNeighbors] = OBSTACLE_COST;
			else c_cost[numNeighbors] = (x != pMin->x && y != pMin->y) ? 7 : 5;
			numNeighbors++;
		}		
	}
	
	if(k_val < pMin->h_cost) {
		for(unsigned i = 0; i < numNeighbors; i++) {
			Cell *pNeighbor = pNeighbors[i];
			if(pNeighbor->list == List_New) continue;
			if(getCost(*pNeighbor) <= val) { // neighbor with h=o (optimal cost)
				unsigned newHCost = c_cost[i];
				if(newHCost < OBSTACLE_COST) newHCost += pNeighbor->h_cost;
				if(pMin->h_cost > newHCost) {
					pMin->h_cost = newHCost;
					pMin->backPtr = pNeighbor;
				}
			}
		}		
	}
	
	if(k_val == pMin->h_cost) {
		for(unsigned i = 0; i < numNeighbors; i++) {
			Cell *pNeighbor = pNeighbors[i];
			unsigned neighborHCost = c_cost[i];
			if(neighborHCost < OBSTACLE_COST) neighborHCost += pMin->h_cost;			
			if(pNeighbor->list == List_New || (pNeighbor->h_cost > neighborHCost) ||
			   (pNeighbor->backPtr == pMin && pNeighbor->h_cost != neighborHCost)) {				   
				pNeighbor->backPtr = pMin;
				insert(*pNeighbor, neighborHCost);				
			}			
		}
	} else {
		for(unsigned i = 0; i < numNeighbors; i++) {
			Cell *pNeighbor = pNeighbors[i];
			unsigned neighborHCost = c_cost[i];
			if(neighborHCost < OBSTACLE_COST) neighborHCost += pMin->h_cost;
			
			if(pNeighbor->list == List_New ||
			   (pNeighbor->backPtr == pMin && pNeighbor->h_cost != neighborHCost)) {
				pNeighbor->backPtr = pMin;
				insert(*pNeighbor, neighborHCost);				
			} else if(pNeighbor->backPtr != pMin) {
				if(pNeighbor->h_cost > neighborHCost) {
					insert(*pMin, pMin->h_cost);										
				} else {
					unsigned hCost = c_cost[i];
					if(hCost < OBSTACLE_COST) hCost += pNeighbor->h_cost;
					if(pMin->h_cost > hCost && pNeighbor->list == List_Closed && val < getCost(*pNeighbor)) {
						insert(*pNeighbor, pNeighbor->h_cost);
					}
				}
			}
		}
	}
	
	return getMinVal();
}

void FocussedDStarPlanner::drawDebugLayer(QPainter &painter, const DebugLayer *layer, const QRect &, qreal) {	
	if(!cells || !openHeap) return;
	
	if(layer == listLayer) {
		painter.drawImage(QPointF(-0.5, -0.5), listMap);
	} else if(layer == backPtrLayer) {		
		const Cell *pCell = cells;
		painter.setPen(QPen(QColor(255, 128, 0), 0));
		painter.setBrush(Qt::NoBrush);
		int h = mapHeight();
		int w = mapWidth();
		for(int y = 0; y < h; y++) {			
			for(int x = 0; x < w; x++) {
				if(pCell->list != List_New) {
					const Cell *pBack = pCell->backPtr;
					if(pBack) {						
						if(pBack->x == x - 1 && pBack->y == y - 1) {
							painter.drawLine(QLineF(x + 0.4, y + 0.4, x - 0.4, y - 0.4));
							painter.drawLine(QLineF(x, y - 0.4, x - 0.4, y - 0.4));
							painter.drawLine(QLineF(x - 0.4, y - 0.4, x - 0.4, y));
						} else if(pBack->x == x - 1 && pBack->y == y) {
							painter.drawLine(QLineF(x + 0.4, y, x - 0.4, y));
							painter.drawLine(QLineF(x, y - 0.4, x - 0.4, y));
							painter.drawLine(QLineF(x - 0.4, y, x, y + 0.4));
						} else if(pBack->x == x - 1 && pBack->y == y + 1) {
							painter.drawLine(QLineF(x + 0.4, y - 0.4, x - 0.4, y + 0.4));
							painter.drawLine(QLineF(x - 0.4, y, x - 0.4, y + 0.4));
							painter.drawLine(QLineF(x - 0.4, y + 0.4, x, y + 0.4));							
						} else if(pBack->x == x && pBack->y == y + 1) {
							painter.drawLine(QLineF(x, y - 0.4, x, y + 0.4));
							painter.drawLine(QLineF(x - 0.4, y, x, y + 0.4));
							painter.drawLine(QLineF(x, y + 0.4, x + 0.4, y));																					
						} else if(pBack->x == x + 1 && pBack->y == y + 1) {
							painter.drawLine(QLineF(x - 0.4, y - 0.4, x + 0.4, y + 0.4));
							painter.drawLine(QLineF(x, y + 0.4, x + 0.4, y + 0.4));
							painter.drawLine(QLineF(x + 0.4, y + 0.4, x + 0.4, y));																												
						} else if(pBack->x == x + 1 && pBack->y == y) {
							painter.drawLine(QLineF(x - 0.4, y, x + 0.4, y));
							painter.drawLine(QLineF(x, y + 0.4, x + 0.4, y));
							painter.drawLine(QLineF(x + 0.4, y, x, y - 0.4));
						} else if(pBack->x == x + 1 && pBack->y == y - 1) {
							painter.drawLine(QLineF(x - 0.4, y + 0.4, x + 0.4, y - 0.4));
							painter.drawLine(QLineF(x + 0.4, y, x + 0.4, y - 0.4));
							painter.drawLine(QLineF(x + 0.4, y - 0.4, x, y - 0.4));
						} else if(pBack->x == x && pBack->y == y - 1) {
							painter.drawLine(QLineF(x, y + 0.4, x, y - 0.4));
							painter.drawLine(QLineF(x - 0.4, y, x, y - 0.4));
							painter.drawLine(QLineF(x, y - 0.4, x + 0.4, y));							
						} else {
							painter.drawLine(QLineF(x - 0.4, y - 0.4, x + 0.4, y + 0.4));
							painter.drawLine(QLineF(x - 0.4, y + 0.4, x + 0.4, y - 0.4));														
						}
					} else painter.drawRect(QRectF(x - 0.25, y - 0.25, 0.5, 0.5));
				}
				pCell++;
			}
		}
	}
}

// move element to the beginning of the heap (lower key values) as far as possible
void FocussedDStarPlanner::heapUp(Cell &cell) {
	if(cell.list != List_Open) return;
	unsigned idx = cell.heapIndex;
	while(idx != 1) {
		unsigned parentIdx = idx >> 1;
		if(*openHeap[idx] < *openHeap[parentIdx]) {
			Cell *temp = openHeap[parentIdx];
			openHeap[parentIdx] = openHeap[idx];
			openHeap[idx] = temp;
			openHeap[parentIdx]->heapIndex = parentIdx;
			openHeap[idx]->heapIndex = idx;
			idx = parentIdx;
		} else break;
	}
}

// move element away from the beginning of the heap (to higher key values) as far as possible
void FocussedDStarPlanner::heapDown(Cell &cell) {
	if(cell.list != List_Open) return;
		
	unsigned idx = cell.heapIndex;
	// Repeat the following until the item sinks to its proper spot in the heap.
	while(1){
		unsigned origIdx = idx;
		unsigned idxChild = idx << 1;		
		if(idxChild <= openListLength) {
			if(*openHeap[idx] >= *openHeap[idxChild]) idx = idxChild;
		}	
		idxChild++;
		if(idxChild <= openListLength) {
			if(*openHeap[idx] >= *openHeap[idxChild]) idx = idxChild;
		}
		
		if(origIdx != idx) {
			Cell *temp = openHeap[idx];
			openHeap[idx] = openHeap[origIdx];
			openHeap[origIdx] = temp;
			openHeap[idx]->heapIndex = idx;
			openHeap[origIdx]->heapIndex = origIdx;
		} else break;
	}
}

void FocussedDStarPlanner::insert(Cell &cell, unsigned h_cost) {
	if(cell.list == List_Open) {
		if(h_cost < cell.k_cost) cell.k_cost = h_cost;
		cell.f_cost = cell.k_cost + dist(cell, *pRobot);
		cell.fB_cost = cell.f_cost + d_curr;
		heapUp(cell);
		heapDown(cell);
	} else {
		if(cell.list == List_New) cell.k_cost = h_cost;
		else cell.k_cost = qMin(cell.h_cost, h_cost);		
		cell.f_cost = cell.k_cost + dist(cell, *pRobot); 
		cell.fB_cost = cell.f_cost + d_curr;
		// insert element
		int idx = ++openListLength;
		openHeap[idx] = &cell;
		cell.heapIndex = idx;
		cell.list = List_Open;
		heapUp(cell);
	}
	cell.h_cost = h_cost;	
	cell.pFocus = pRobot;	
}
void FocussedDStarPlanner::dumpCell(const Cell &cell) {
	printf("INFO: Cell (%d, %d)\n", cell.x, cell.y);
	if(cell.blocked) printf(" - blocked\n");
	printf(" - List = %s\n", cell.list == List_New ? "NEW" :
							 cell.list == List_Open ? "OPEN":
							 cell.list == List_Closed ? "CLOSED" : "<unknown>");
	if(cell.list == List_Closed || cell.list == List_Open) printf(" - k_cost = %u, h_cost = %u, f_cost = %u, fB_cost = %u\n", cell.k_cost, cell.h_cost, cell.f_cost, cell.fB_cost);
}

void FocussedDStarPlanner::dumpOpenHeap() const {
	printf("OPEN list Heap Dump\n");
	dumpOpenHeapLayer(1, 1);
}
void FocussedDStarPlanner::dumpOpenHeapLayer(unsigned index, unsigned level) const {
	printf("%*s(%u, %u, %u) - cell (%d, %d)\n", 3 * level, "", openHeap[index]->fB_cost, openHeap[index]->f_cost, openHeap[index]->k_cost, openHeap[index]->x, openHeap[index]->y);
	index <<= 1;
	if(index <= openListLength) dumpOpenHeapLayer(index, level + 1);
	index++;
	if(index <= openListLength) dumpOpenHeapLayer(index, level + 1);	
}
