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

#include "dstarplanner.h"
#include <new>
#include <cstdio>
#include <QPainter>
#include <QAction>

#define OBSTACLE_COST	2000000000U

DStarPlanner::DStarPlanner(QObject *parent):
	AbstractPlanner(parent),
	cells(NULL), openHeap(NULL), openListLength(0),
	listLayer(NULL), backPtrLayer(NULL),
	inhibitStep(false)
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

DStarPlanner::~DStarPlanner() {
	freeData();
}
	
void DStarPlanner::freeData() {
	if(cells) {
		delete[] cells;
		cells = NULL;
	}
	if(openHeap) {
		delete[] openHeap;
		openHeap = NULL;
	}	
}


void DStarPlanner::initMap(const QImage &map, const QRect &updateRegion) {	
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
				pCell->list = List_New;
				pCell->heapIndex = 0;
				pCell->h_cost = 0;
				pCell->blocked = (*pCost++ > 0);			
				pCell++;
			}			
		}	
	} else {
		// It's a map update: incorporate cost changes
		// --> This implements MODIFY-COST from the Pseudo-Code in Stentz' Paper

		for(int y = updateRegion.top(); y <= updateRegion.bottom(); y++) {
			const unsigned char *pCost = (const unsigned char *)map.scanLine(y) + updateRegion.left();
			int w = mapWidth();
			int h = mapHeight();
			
			Cell *pCell = cells + w * y + updateRegion.left();
			for(int i = 0; i < updateRegion.width(); i++) {
				bool newBlocked = (*pCost++ > 0);
				if(newBlocked != pCell->blocked) {
					pCell->blocked = newBlocked;
					// add the changed cell itself to the OPEN list
					if(pCell->list == List_Closed) insert(pCell, pCell->h_cost);
										
					if(!pCell->blocked) {
						// if a cell has been unblocked, add all neighbors to the open list.
						// Note: In the paper, only arc cost changes are mentioned, but we are working on cells, i.e. if
						// chaning a cell's cost this influences all arcs from this cell to its neighbors.
						for(int iy = pCell->y - 1; iy <= pCell->y + 1; iy++) {
							if((unsigned)iy >= (unsigned)h) continue;
							for(int ix = pCell->x - 1; ix <= pCell->x + 1; ix++) {
								if((unsigned)ix >= (unsigned)w) continue;
								Cell *pNeighbor = cells + iy * w + ix;
								if(pNeighbor->list == List_Closed) insert(pNeighbor, pNeighbor->h_cost);
							}
						}
					}					
				}
				pCell++;			
			}		
		}
	}
}

void DStarPlanner::singleSteppingToggled(bool enabled) {
	if(!enabled) {
		doCalculatePath(0, false);	
		emit dataChanged();
	}
}

void DStarPlanner::doSingleStep() {
	doCalculatePath(0, true);
	// Inform GUI for redrawing
	emit dataChanged();
}

void DStarPlanner::calculatePath(InputUpdates updates) {
	inhibitStep = singleSteppingAction->isChecked();			
	doCalculatePath(updates, inhibitStep);
}

void DStarPlanner::doCalculatePath(InputUpdates updates, bool singleStep) {
	if(!cells || !openHeap) {
		setError("Planner memory allocation error");
		return;
	}
	
	unsigned width = mapWidth();
	unsigned height = mapHeight();
	// prepare image that shows list membership (size & color table)
	if(listMap.size() != mapSize()) {
		listMap = QImage(mapSize(), QImage::Format_Indexed8);
		listMap.setColorTable(QVector<QRgb>() << qRgba(0, 0, 0, 0) << qRgba(0, 255, 255, 192) << qRgba(255, 255, 0, 128) << qRgba(255, 192, 0, 192) << qRgb(0, 200, 0));
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

		openListLength = 1;
		pGoal->k_cost = pGoal->h_cost = 0;
		pGoal->list = List_Open;
		pGoal->heapIndex = 1;
		openHeap[1] = pGoal;
	}
	
	bool success = true;
	
	// processState loop		
	if(inhibitStep) {
		inhibitStep = false;
		setError("Single stepping enabled...");
		success = false;
	} else {	
		unsigned kMin = getKMin();
		if(pStart->list == List_New || kMin < pStart->h_cost) {
			do {				
				kMin = processState(false);	
				
				if(pStart->list != List_New && kMin >= pStart->h_cost) break;
				
				if(kMin >= OBSTACLE_COST) {
					setError("No Path found");
					success = false;
					break;
				}
				if(singleStep) {
					printf("kMin = %d, pStart->h_cost = %d\n", kMin, pStart->h_cost);
					success = false;
					setError("Not yet ready...");
					break;
				}				
			} while(true);
		}
		
		if(success && (pStart->list == List_New || pStart->h_cost >= OBSTACLE_COST)) {
			// may reach here, if no path has been found in a previous step and no open cells (with costs below OBSTACLE) exist
			setError("No Path found");
			success = false;
		}
	}

	// prepare debug layers
	const Cell *pCell = cells;
	for(unsigned y = 0; y < height; y++) {
		unsigned char *pMap = listMap.scanLine(y);
		for(unsigned x = 0; x < width; x++) {
			switch(pCell->list) {
			case List_Closed: *pMap++ = 2; break;
			case List_Open: *pMap++ = pCell->k_cost >= OBSTACLE_COST ? 3 : 1; break;
			default: *pMap++ = 0;
			}
			pCell++;
		}
	}
	if(openListLength >= 1) listMap.setPixel(openHeap[1]->x, openHeap[1]->y, 4);
	
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

// Heart of the DStar planner, implemented according to the pseudocode in the A. Stentz' ICRA'94 paper

unsigned DStarPlanner::processState(bool singleStep) {
	// error if open list is empty
	if(openListLength < 1) return OBSTACLE_COST;
	
	// remove first entry from the open list
	Cell *pMin = openHeap[1];
	pMin->list = List_Closed;
	pMin->heapIndex = 0;	
	if(--openListLength){
		openHeap[1] = openHeap[openListLength + 1]; //move the last item in the heap up to slot #1
		openHeap[1]->heapIndex = 1;
		heapDown(openHeap[1]);
	}		
	unsigned oldKMin = pMin->k_cost;
	if(singleStep) printf("### processState for (%d, %d), h_cost = %u, oldKMin = k_cost = %u ###\n", pMin->x, pMin->y, pMin->h_cost, oldKMin);	
	
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
			else c_cost[numNeighbors] = (x != pMin->x && y != pMin->y) ? 14 : 10;
			numNeighbors++;
		}		
	}
	
	if(oldKMin < pMin->h_cost) {
		for(unsigned i = 0; i < numNeighbors; i++) {
			Cell *pNeighbor = pNeighbors[i];
			if(pNeighbor->list == List_New) continue;
			if(pNeighbor->h_cost <= oldKMin) { // neighbor with h=o (optimal cost)
				unsigned newHCost = c_cost[i];
				if(newHCost < OBSTACLE_COST) newHCost += pNeighbor->h_cost;
				if(pMin->h_cost > newHCost) {
					pMin->h_cost = newHCost;
					pMin->backPtr = pNeighbor;
				}
			}
		}		
	}
	
	if(oldKMin == pMin->h_cost) {
		for(unsigned i = 0; i < numNeighbors; i++) {
			Cell *pNeighbor = pNeighbors[i];
			unsigned neighborHCost = c_cost[i];
			if(neighborHCost < OBSTACLE_COST) neighborHCost += pMin->h_cost;			
			if(pNeighbor->list == List_New || (pNeighbor->h_cost > neighborHCost) ||
			   (pNeighbor->backPtr == pMin && pNeighbor->h_cost != neighborHCost)) {				   
				pNeighbor->backPtr = pMin;
				insert(pNeighbor, neighborHCost);				
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
				insert(pNeighbor, neighborHCost);				
			} else if(pNeighbor->backPtr != pMin) {
				if(pNeighbor->h_cost > neighborHCost) {
					insert(pMin, pMin->h_cost);										
				} else {
					unsigned hCost = c_cost[i];
					if(hCost < OBSTACLE_COST) hCost += pNeighbor->h_cost;
					if(pMin->h_cost > hCost && pNeighbor->list == List_Closed && pNeighbor->h_cost > oldKMin) {
						insert(pNeighbor, pNeighbor->h_cost);
					}
				}
			}
		}
	}
	
	return getKMin();
}

unsigned DStarPlanner::getKMin() const {
	if(openListLength == 0) return OBSTACLE_COST;
	else return openHeap[1]->k_cost;
}

// move element to the beginning of the heap (lower key values) as far as possible
void DStarPlanner::heapUp(Cell *pCell) {
	if(pCell->list != List_Open) return;
	unsigned idx = pCell->heapIndex;
	while(idx != 1) {
		unsigned parentIdx = idx >> 1;
		if(openHeap[idx]->k_cost < openHeap[parentIdx]->k_cost) {
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
void DStarPlanner::heapDown(Cell *pCell) {
	if(pCell->list != List_Open) return;
		
	unsigned idx = pCell->heapIndex;
	//	Repeat the following until the item  sinks to its proper spot in the heap.
	while(1){		
		unsigned origIdx = idx;
		unsigned idxChild = idx << 1;		
		if(idxChild <= openListLength) {
			if(openHeap[idx]->k_cost >= openHeap[idxChild]->k_cost) idx = idxChild;
		}	
		idxChild++;
		if(idxChild <= openListLength) {
			if(openHeap[idx]->k_cost >= openHeap[idxChild]->k_cost) idx = idxChild;
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

void DStarPlanner::insert(Cell *pCell, unsigned h_cost) {
	if(pCell->list == List_Open) {
		if(h_cost < pCell->k_cost) pCell->k_cost = h_cost;
		pCell->h_cost = h_cost;
		heapUp(pCell);
		heapDown(pCell);
	} else {
		if(pCell->list == List_New) pCell->h_cost = pCell->k_cost = h_cost;
		else {
			pCell->k_cost = qMin(pCell->h_cost, h_cost);
			pCell->h_cost = h_cost;
		}
		// insert element
		int idx = ++openListLength;
		openHeap[idx] = pCell;
		pCell->heapIndex = idx;
		pCell->list = List_Open;		
		pCell->k_cost = qMin(pCell->k_cost, h_cost);
		heapUp(pCell);
	}	
}
void DStarPlanner::dumpCell(const Cell *pCell) {
	if(!pCell) return;
	printf("INFO: Cell (%d, %d)\n", pCell->x, pCell->y);
	if(pCell->blocked) printf(" - blocked\n");
	printf(" - List = %s\n", pCell->list == List_New ? "NEW" :
							 pCell->list == List_Open ? "OPEN":
							 pCell->list == List_Closed ? "CLOSED" : "<unknown>");
	if(pCell->list == List_Closed || pCell->list == List_Open) printf(" - k_cost = %u, h_cost = %u\n", pCell->k_cost, pCell->h_cost);
}

void DStarPlanner::dumpOpenHeap() const {
	printf("OPEN list Heap Dump\n");
	dumpOpenHeapLayer(1, 1);
}
void DStarPlanner::dumpOpenHeapLayer(unsigned index, unsigned level) const {
	printf("%*s%d - cell (%d, %d)\n", level, "", openHeap[index]->k_cost, openHeap[index]->x, openHeap[index]->y);
	index <<= 1;
	if(index <= openListLength) dumpOpenHeapLayer(index, level + 1);
	index++;
	if(index <= openListLength) dumpOpenHeapLayer(index, level + 1);	
}

void DStarPlanner::drawDebugLayer(QPainter &painter, const DebugLayer *layer, const QRect &, qreal) {	
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
