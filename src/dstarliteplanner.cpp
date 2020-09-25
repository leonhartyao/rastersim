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

#include "dstarliteplanner.h"
#include <new>
#include <climits>
#include <cstdio>
#include <QPainter>
#include <QAction>
#include <QActionGroup>
#include <QSignalMapper>
#include <QFile>

#define OBSTACLE_COST	(UINT_MAX - 10000000)

DStarLitePlanner::DStarLitePlanner(QObject *parent):
	AbstractPlanner(parent),
	cells(NULL), openHeap(NULL), openListLength(0),
	listLayer(NULL), costLayer(NULL), backPtrs(NULL),
	inhibitStep(false),
	saveStateCounter(-1) // set to -1 to disable state saving
{
	singleSteppingAction = new QAction(tr("Stepping"), this);
	singleSteppingAction->setCheckable(true);
	addAction(singleSteppingAction);
	
	QSignalMapper *steppingMapper = new QSignalMapper(this);
	connect(steppingMapper, SIGNAL(mapped(int)), this, SLOT(doSteps(int)));
	singleStepGroup = new QActionGroup(this);
	connect(singleSteppingAction, SIGNAL(toggled(bool)), singleStepGroup, SLOT(setEnabled(bool)));
	connect(singleSteppingAction, SIGNAL(toggled(bool)), this, SLOT(singleSteppingToggled(bool)));
	
	for(int i = 1; i <= 100000; i *= 10) {
		bool kFlag = (i > 1000);
		QAction *stepAction = new QAction(QString(tr(kFlag ? "%1k" : "%1")).arg(kFlag ? i / 1000 : i), this);
		singleStepGroup->addAction(stepAction);
		connect(stepAction, SIGNAL(triggered(bool)), steppingMapper, SLOT(map()));
		steppingMapper->setMapping(stepAction, i);
		addAction(stepAction);
	}
		
	loadStateAction = new QAction(tr("Load state"), this);
	connect(loadStateAction, SIGNAL(triggered(bool)), this, SLOT(loadState()));
	loadMapAction = new QAction(tr("Load map"), this);
	connect(loadMapAction, SIGNAL(triggered(bool)), this, SLOT(loadMapState()));
	addAction(loadStateAction);
	addAction(loadMapAction);
}

DStarLitePlanner::~DStarLitePlanner() {
	freeData();
}
	
void DStarLitePlanner::freeData() {
	if(cells) {
		delete[] cells;
		cells = NULL;
	}
	if(openHeap) {
		delete[] openHeap;
		openHeap = NULL;
	}	
}
	
void DStarLitePlanner::initMap(const QImage &map, const QRect &updateRegion) {
	unsigned h = mapHeight();
	unsigned w = mapWidth();

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
		
		// initialized cells and neighborhood patterns
		Cell *pCell = cells;
		for(unsigned y = 0; y < h; y++) {
			const unsigned char *pCost = (const unsigned char *)map.scanLine(y);
			for(unsigned x = 0; x < w; x++) {
				pCell->x = x;
				pCell->y = y;
				pCell->neighborhoodIndex = 0;
				pCell->blocked = (*pCost++ > 0);			
				pCell++;
			}			
		}
		Cell *pMin = cells;
		Cell *pMax = cells + (h - 1) * w;
		for (unsigned i = 0; i < w; i++) {
			pMin->neighborhoodIndex |= YMinEdge;
			pMax->neighborhoodIndex |= YMaxEdge;
			pMin++; pMax++;
		}
		pMin = cells;
		pMax = cells + w - 1;
		for(unsigned i = 0; i < h; i++) {
			pMin->neighborhoodIndex |= XMinEdge;
			pMax->neighborhoodIndex |= XMaxEdge;
			pMin += w;
			pMax += w;
		}
		
		neighborhoods = std::vector<Neighborhood>(16, Neighborhood(1));
		for(int y = -1; y <= 1; y++) {
			for(int x = -1; x <= 1; x++) {
				if (x == 0 && y == 0) continue;
				for(unsigned i = 0; i < 16; i++) {
					if((i & XMinEdge) && (x == -1)) continue;
					if((i & XMaxEdge) && (x == 1)) continue;
					if((i & YMinEdge) && (y == -1)) continue;
					if((i & YMaxEdge) && (y == 1)) continue;
					neighborhoods[i].push_back(NeighborSpec(y * w + x, x == 0 || y == 0 ? 5 : 7));
				}
			}
		}
		
	} else {
		// It's a map update: incorporate cost changes
		for(int y = updateRegion.top(); y <= updateRegion.bottom(); y++) {
			const unsigned char *pCost = (const unsigned char *)map.scanLine(y) + updateRegion.left();
			Cell *pCell = cells + w * y + updateRegion.left();
			
			for(int i = 0; i < updateRegion.width(); i++) {
				bool newBlocked = (*pCost++ > 0);
				if(newBlocked != pCell->blocked) {
					
					pCell->blocked = newBlocked;
					const Neighborhood &neighborhood = neighborhoods.at(pCell->neighborhoodIndex);					
					if(pCell->blocked) {
						/*printf("New blocked cell (%u, %u)\n", pCell->x, pCell->y);
						for(unsigned i = 1; i < neighborhood.size(); i++) {							
							Cell *pNeighbor = pCell + neighborhood[i].ptrOffset;
							printf(" examining neighbor (%u, %u)\n", pNeighbor->x, pNeighbor->y);
							if(pNeighbor == pGoal) {
								printf("  is Goal\n");
								continue;
							}
							if(pNeighbor->blocked) {
								printf("  is blocked\n");
								continue;
							}
							unsigned testCost = pCell->g_cost;
							if(testCost < OBSTACLE_COST) testCost += neighborhood[i].baseCost;
							if(pNeighbor->rhs != testCost) {
								printf("  has no 'backptr' to cell\n");
								continue;
							}							
								
							const Neighborhood &neighborhood2 = neighborhoods.at(pNeighbor->neighborhoodIndex);
							unsigned newRhs = OBSTACLE_COST;
							for(unsigned j = 1; j < neighborhood2.size(); j++) {
								const Cell *pNeighbor2 = pNeighbor + neighborhood2[j].ptrOffset;
								if(pNeighbor2->blocked) continue;
								unsigned rhs = pNeighbor2->g_cost;								
								if(rhs < OBSTACLE_COST) rhs += neighborhood2[j].baseCost;
								//printf("   rhs from neighbor (%u, %u) is %u\n", pNeighbor2->x, pNeighbor2->y, rhs);
								if(rhs < newRhs) newRhs = rhs;
							}
							printf("  new cost = %u vs. %u\n", newRhs, pNeighbor->rhs);
							if(newRhs != pNeighbor->rhs) {
								pNeighbor->rhs = newRhs;
								updateVertex(pNeighbor);	
							}							
						}
						
						printf(" removing this cell from list\n");
						pCell->rhs = pCell->g_cost = OBSTACLE_COST;
						remove(*pCell);*/
						
						for(unsigned i = 1; i < neighborhood.size(); i++) {
							Cell *pNeighbor = pCell + neighborhood[i].ptrOffset;
							if(pNeighbor == pGoal) continue;							
							if(pNeighbor->blocked) continue; // if everything is consistent, we cannot decrease a blocked cell's cost
							unsigned newRhs = OBSTACLE_COST;
							const Neighborhood &neighborhood2 = neighborhoods.at(pNeighbor->neighborhoodIndex);
							for(unsigned j = 1; j < neighborhood2.size(); j++) {
								const Cell *pNeighbor2 = pNeighbor + neighborhood2[j].ptrOffset;
								if(pNeighbor2->blocked) continue;
								unsigned rhs = pNeighbor2->g_cost;
								if(rhs < OBSTACLE_COST) rhs += neighborhood2[j].baseCost;
								if(rhs < newRhs) newRhs = rhs;									
							}
							if(pNeighbor->rhs != newRhs) {
								pNeighbor->rhs = newRhs;								
								updateVertex(pNeighbor);							
							}
						}
						pCell->rhs = pCell->g_cost = OBSTACLE_COST;
						updateVertex(pCell);
											
					} else {
						unsigned newCellRhs = pCell->rhs;
						for(unsigned i = 1; i < neighborhood.size(); i++) {
							Cell *pNeighbor = pCell + neighborhood[i].ptrOffset;
							if(pNeighbor == pGoal || pNeighbor->blocked) continue;
							
							unsigned newRhs = pNeighbor->g_cost;
							if(newRhs < OBSTACLE_COST) newRhs += neighborhood[i].baseCost;
							if(newRhs < newCellRhs) newCellRhs = newRhs;
							
							newRhs = pCell->g_cost;
							if(newRhs < OBSTACLE_COST) newRhs += neighborhood[i].baseCost;
							if(newRhs < pNeighbor->rhs) {
								pNeighbor->rhs = newRhs;
								updateVertex(pNeighbor);
							}							
						}
						if(newCellRhs < pCell->rhs) {
							pCell->rhs = newCellRhs;
							updateVertex(pCell);
						}
					}					
				}
				pCell++;			
			}		
		}
	}
}

bool DStarLitePlanner::computeShortestPath(unsigned maxSteps) {
	bool complete = true;
	// make sure the start key is initialized
	//pStart->key = pStart->calculateKey(*pStart, k_m);
	
	unsigned step = 0;
	
	while(openListLength) {
		Cell *pCell = openHeap[1];

		unsigned k2Start = qMin(pStart->g_cost, pStart->rhs);		
		if(!(pCell->key < Key(k2Start + k_m, k2Start) || pStart->rhs > pStart->g_cost)) break;

		if(maxSteps && (++step > maxSteps)) {
			complete = false;
			break;
		}
		
		// remove first element
		listMap.setPixel(pCell->x, pCell->y, 1);
		
		Key correctKey = pCell->calculateKey(*pStart, k_m);
		if(pCell->key < correctKey) {
			pCell->key = correctKey;
			insert(*pCell);
		} else if(pCell->g_cost > pCell->rhs) {
			pCell->g_cost = pCell->rhs;
			remove(*pCell);
			if(!pCell->blocked) { // should not happen
				const Neighborhood &neighborhood = neighborhoods.at(pCell->neighborhoodIndex);
				for(unsigned i = 1; i < neighborhood.size(); i++) {				
					Cell *pNeighbor = pCell + neighborhood[i].ptrOffset;
					if(pNeighbor->blocked || pNeighbor == pGoal) continue;
					unsigned newCost = pCell->g_cost;
					if(newCost < OBSTACLE_COST) newCost += neighborhood[i].baseCost;
					if(pNeighbor->rhs > newCost) {
						pNeighbor->rhs = newCost;					
						updateVertex(pNeighbor);
					}
				}
			}
		} else {
			unsigned g_old = pCell->g_cost;			
			pCell->g_cost = OBSTACLE_COST;
			
			const Neighborhood &neighborhood = neighborhoods.at(pCell->neighborhoodIndex);
			/*if(!pCell->blocked && pCell != pGoal) {				
				unsigned newRhs = pCell->rhs;
				for(unsigned i = 0; i < neighborhood.size(); i++) {
					const Cell *pNeighbor = pCell + neighborhood[i].ptrOffset;
					unsigned rhs = pNeighbor->g_cost;
					if(rhs < OBSTACLE_COST) rhs += neighborhood[i].baseCost;
					if(rhs < newRhs) newRhs = rhs;
				}
				if(pCell->rhs != newRhs) {
					pCell->rhs = newRhs;
					updateVertex(pCell);
				}
			}
			
			for(unsigned i = 1; i < neighborhood.size(); i++) {
				Cell *pNeighbor = pCell + neighborhood[i].ptrOffset;
				if(pNeighbor->blocked && pNeighbor != pCell) continue;
				
				unsigned testCost = g_old;
				if(testCost < OBSTACLE_COST) testCost += neighborhood[i].baseCost;
				if(pNeighbor->rhs != testCost) continue;
				
				const Neighborhood &neighborhood2 = neighborhoods.at(pNeighbor->neighborhoodIndex);
				for(unsigned j = 1; j < neighborhood2.size(); j++) {
					const Cell *pNeighbor2 = pNeighbor + neighborhood2[j].ptrOffset;								
					unsigned newCost = pNeighbor2->g_cost;
					if(newCost < OBSTACLE_COST) {
						newCost += neighborhood2[j].baseCost;
						if(newCost < pNeighbor->rhs) pNeighbor->rhs = newCost;
					}
				}
				updateVertex(pNeighbor);
			}*/
			
			for(unsigned i = 0; i < neighborhood.size(); i++) {
				Cell *pNeighbor = pCell + neighborhood[i].ptrOffset;
				if(pNeighbor->blocked || pNeighbor == pGoal) continue;

				unsigned testCost = g_old;
				if(testCost < OBSTACLE_COST) testCost += neighborhood[i].baseCost;
				
				if(pNeighbor == pCell || pNeighbor->rhs == testCost) {
					unsigned newRhs = OBSTACLE_COST;
					const Neighborhood &neighborhood2 = neighborhoods.at(pNeighbor->neighborhoodIndex);
					for(unsigned j = 1; j < neighborhood2.size(); j++) {							
						const Cell *pNeighbor2 = pNeighbor + neighborhood2[j].ptrOffset;
						if(pNeighbor2->blocked) continue;
						unsigned rhs = pNeighbor2->g_cost;
						if(rhs < OBSTACLE_COST) rhs += neighborhood2[j].baseCost;
						if(rhs < newRhs) newRhs = rhs;
					}
					pNeighbor->rhs = newRhs;
				}				   
				updateVertex(pNeighbor);				
			}
		}
	}	
	
	return complete;
}

void DStarLitePlanner::singleSteppingToggled(bool enabled) {
	if(!enabled) {
		doCalculatePath(0, 0);	
		emit dataChanged();
	}
}

void DStarLitePlanner::doSteps(int max) {
	doCalculatePath(0, max);
	// Inform GUI for redrawing
	emit dataChanged();
}

void DStarLitePlanner::calculatePath(InputUpdates updates) {	
	inhibitStep = singleSteppingAction->isChecked();			
	doCalculatePath(updates, inhibitStep);	
}
	
void DStarLitePlanner::doCalculatePath(InputUpdates updates, unsigned maxSteps) {
	if(!cells || !openHeap) {
		setError("Planner memory allocation error");
		return;
	}
	
	// some preparations...
	unsigned w = mapWidth();
	unsigned h = mapHeight();

	if(listMap.size() != mapSize()) {
		listMap = QImage(mapSize(), QImage::Format_Indexed8);
		listMap.setColorTable(QVector<QRgb>() << qRgba(0, 0, 0, 0) << qRgba(255, 255, 0, 192) << qRgba(255, 128, 0, 192) << qRgba(0, 255, 255, 192) << qRgba(255, 0, 255, 192) << qRgb(0, 200, 0));
	}
	listMap.fill(0);
			
	QPoint startPos = start().pos().toPoint();
	QPoint goalPos = goal().pos().toPoint();
	pStart = cells + (int)startPos.y() * w + (int)startPos.x();
	pGoal = cells + (int)goalPos.y() * w + (int)goalPos.x();

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
		Cell *pEnd = cells + w * h;
		Cell *pCell = cells;	
		while(pCell != pEnd) {
			pCell->heapIndex = 0;
			pCell->rhs = pCell->g_cost = OBSTACLE_COST;
			++pCell;
		}

		k_m = 0;
		pRobot = pStart;

		openListLength = 1;
		pGoal->rhs = 0;
		pGoal->heapIndex = 1;
		openHeap[1] = pGoal;
		pGoal->key = pGoal->calculateKey(*pStart, k_m);
	}

	bool success = true;
	if(inhibitStep) {
		inhibitStep = false;
		setError("Single stepping enabled...");
		success = false;
	} else {	
		if(pRobot != pStart) {
			k_m += h_cost(*pStart, *pRobot);
			pRobot = pStart;
		}
		// compute path
		success = computeShortestPath(maxSteps);
	}
	
	// prepare debug layers
	doDebugAndPathExtract(success);
}

void DStarLitePlanner::doDebugAndPathExtract(bool pathExtract) {
	unsigned h = mapHeight();
	unsigned w = mapWidth();
	
	const Cell *pCell = cells;
	for(unsigned y = 0; y < h; y++) {
		unsigned char *pMap = listMap.scanLine(y);
		for(unsigned x = 0; x < w; x++) {
			if(pCell->heapIndex > 0) {
				if(pCell->g_cost != pCell->rhs) *pMap = 4;
				else *pMap = 3;
			} else if(*pMap > 0) {
				if(pCell->g_cost != pCell->rhs) *pMap = 2;
				else *pMap = 1;
			}
			pMap++;
			pCell++;
		}
	}
	if(openListLength >= 1) listMap.setPixel(openHeap[1]->x, openHeap[1]->y, 5);
	if(!listLayer) addDebugLayer(listLayer = new DebugLayer(tr("Lists (cyan = open, yellow = touched)")));
	if(!backPtrs) {
		addDebugLayer(backPtrs = new DebugLayer(tr("Backpointers"), 1));
		backPtrs->setMinimumZoomFactor(6.0);
	}
	if(!costLayer) {
		addDebugLayer(costLayer = new DebugLayer(tr("cost (blue = g, red = rhs"), 1));	
		costLayer->setMinimumZoomFactor(16.0);
	}
	
	if (pathExtract) {
		if(saveStateCounter >= 0) {
			QString fileName;
			fileName.sprintf("dstarlite%05u.bin", saveStateCounter++);
			saveState(fileName);			
		}
		
		// extract path
		if(pStart->rhs < OBSTACLE_COST) {
			Path p;
			Cell *pCell = pStart;
			bool success = true;
			unsigned pathLength = 0;
			while(true) {
				p.push_back(QPointF(pCell->x, pCell->y));
				if(pCell == pGoal) break;
				
				if(++pathLength > 100000) {
					setError("Path too long\n");
					success = false;
					break;
				}
				
				const Neighborhood &neighborhood = neighborhoods[pCell->neighborhoodIndex];
				Cell *pNextCell = NULL;
				unsigned minCost = OBSTACLE_COST;
				for(unsigned i = 1; i < neighborhood.size(); i++) {
					Cell *pNeighbor = pCell + neighborhood[i].ptrOffset;
					if(!pNeighbor->blocked && pNeighbor->g_cost < OBSTACLE_COST) {
						unsigned cost = pNeighbor->g_cost + neighborhood[i].baseCost;
						if(cost < minCost) {
							minCost = cost;
							pNextCell = pNeighbor;
						}
					}
				}
				
				if(!pNextCell) {
					setError("Path blocked\n");
					success = false;
					break;
				}
				pCell = pNextCell;
			}
			
			if(success) setPath(p);
		} else setError("No Path found");
	}
}

void DStarLitePlanner::updateVertex(Cell *pCell) {
	if(pCell->g_cost != pCell->rhs) {
		pCell->key = pCell->calculateKey(*pStart, k_m);
		insert(*pCell);		
	} else remove(*pCell);	
}

// move element to the beginning of the heap (lower key values) as far as possible
void DStarLitePlanner::heapUp(Cell &cell) {
	if(!cell.heapIndex) return;
	unsigned idx = cell.heapIndex;
	while(idx != 1) {
		unsigned parentIdx = idx >> 1;
		if(openHeap[idx]->key < openHeap[parentIdx]->key) {
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
void DStarLitePlanner::heapDown(Cell &cell) {		
	if(!cell.heapIndex) return;
	
	unsigned idx = cell.heapIndex;
	// Repeat the following until the item sinks to its proper spot in the heap.
	while(1){
		unsigned origIdx = idx;
		unsigned idxChild = idx << 1;		
		if(idxChild <= openListLength) {
			if(openHeap[idx]->key >= openHeap[idxChild]->key) idx = idxChild;
		}	
		idxChild++;
		if(idxChild <= openListLength) {
			if(openHeap[idx]->key >= openHeap[idxChild]->key) idx = idxChild;
		}
			
		if(origIdx == idx) break;		
		Cell *temp = openHeap[idx];
		openHeap[idx] = openHeap[origIdx];
		openHeap[origIdx] = temp;
		openHeap[idx]->heapIndex = idx;
		openHeap[origIdx]->heapIndex = origIdx;
	}
}



void DStarLitePlanner::insert(Cell &cell) {
	if(cell.heapIndex) { 
		// update position in heap
		heapDown(cell);
		heapUp(cell);
	} else { 
		// insert element
		openListLength++;
		openHeap[openListLength] = &cell;
		cell.heapIndex = openListLength;
		heapUp(cell);
	}
}
void DStarLitePlanner::remove(Cell &cell) {
	if(!cell.heapIndex) return;

	Cell *lastCell = openHeap[openListLength--];
	if(openListLength) {
		unsigned idx = cell.heapIndex;		
		openHeap[idx] = lastCell;
		lastCell->heapIndex = idx;		
		heapUp(*lastCell);
		heapDown(*lastCell);
	}	
	cell.heapIndex = 0;		
}

void DStarLitePlanner::checkHeap() const {
	if(openListLength < 1) return;
	unsigned invalidHeapIndex = checkHeapLayer(1, Key());
	if(invalidHeapIndex > 0) {
		printf("Heap corruption at index %u\n", invalidHeapIndex);
		dumpHeap(invalidHeapIndex);
	}		
	
}

unsigned DStarLitePlanner::checkHeapLayer(unsigned index, Key key) const {
	Key myKey = openHeap[index]->key;
	if(myKey < key) return index;
	
	unsigned result = 0;
	index <<= 1;	
	if(index <= openListLength) result = checkHeapLayer(index, myKey);
	if(result == 0) {
		index++;
		if(index <= openListLength) result = checkHeapLayer(index, myKey);
	}
	return result;
}

void DStarLitePlanner::dumpHeap(unsigned mark) const {
	printf("OPEN list Heap Dump (size = %d)\n", openListLength);
	dumpHeapLayer(1, 1, mark);
}
void DStarLitePlanner::dumpHeapLayer(unsigned index, unsigned level, unsigned mark) const {
	unsigned printDist = 3 * level;
	if(mark == index) {
		while(printDist) {
			printf("!");
			printDist--;
		}
	}	
	printf("%*s(%u, %u) - cell (%u, %u)\n", printDist, "", openHeap[index]->key.k1, openHeap[index]->key.k2, openHeap[index]->x, openHeap[index]->y);
	index <<= 1;
	if(index <= openListLength) dumpHeapLayer(index, level + 1, mark);
	index++;
	if(index <= openListLength) dumpHeapLayer(index, level + 1, mark);	
}

void DStarLitePlanner::drawDebugLayer(QPainter &painter, const DebugLayer *layer, const QRect &visibleArea, qreal zoomFactor) {
	if(!cells || !openHeap) return;
	
	if(layer == listLayer) {
		painter.drawImage(QPointF(-0.5, -0.5), listMap);
		if(openListLength) {
			QPen nextPen(QColor(0, 200, 0));
			nextPen.setCosmetic(true);
			nextPen.setWidth(2);
			painter.setPen(nextPen);
			qreal radius = qMax(10.0 / zoomFactor, 1.0);
			painter.drawEllipse(QPointF(openHeap[1]->x, openHeap[1]->y), radius, radius);
		}
		
	} else if(layer == costLayer) {
		QTransform t = painter.transform();
		painter.resetTransform();
		painter.setRenderHint(QPainter::TextAntialiasing, false);		
		QFont font = painter.font();		
		font.setPointSize(8);		
		painter.setFont(font);
		painter.setPen(QColor(32, 32, 255));
		unsigned xStart = visibleArea.left();
		unsigned yStart = visibleArea.top();
		unsigned xEnd = xStart + visibleArea.width();
		unsigned yEnd = yStart + visibleArea.height();
		const Cell *pCell = cells + yStart * mapWidth() + xStart;
		
		QPen gCostPen(QColor(32, 32, 255));
		QPen rhsPen(QColor(160, 0, 0));
		for(unsigned y = yStart; y < yEnd; y++) {
			for(unsigned x = xStart; x < xEnd; x++) {
				painter.setPen(gCostPen);
				painter.drawText(t.mapRect(QRect(x - 1, y, 2, 1)), Qt::AlignHCenter | Qt::AlignBottom, 
								 pCell->g_cost < OBSTACLE_COST ? QString::number(pCell->g_cost) : "x");
				painter.setPen(rhsPen);
				painter.drawText(t.mapRect(QRect(x - 1, y - 1, 2, 1)), Qt::AlignHCenter | Qt::AlignTop, 
								 pCell->rhs < OBSTACLE_COST ? QString::number(pCell->rhs) : "x");
				pCell++;
			}
			pCell += mapWidth() - (xEnd - xStart);
		}		
		painter.setTransform(t);
		
	} else if(layer == backPtrs) {
		painter.setPen(QPen(QColor(255, 128, 0), 0));
		painter.setBrush(Qt::NoBrush);
		unsigned xStart = visibleArea.left();
		unsigned yStart = visibleArea.top();
		unsigned xEnd = xStart + visibleArea.width();
		unsigned yEnd = yStart + visibleArea.height();
		const Cell *pCell = cells + yStart * mapWidth() + xStart;

		for(unsigned y = yStart; y < yEnd; y++) {			
			for(unsigned x = xStart; x < xEnd; x++) {
				if(pCell != pGoal) {
					const Cell *pBack = NULL;
					const Neighborhood &neighborhood = neighborhoods.at(pCell->neighborhoodIndex);
					unsigned minCost = OBSTACLE_COST;						
					for(unsigned i = 1; i < neighborhood.size(); i++) {
						const Cell *pNeighbor = pCell + neighborhood[i].ptrOffset;
						if(pNeighbor->blocked || pNeighbor->g_cost >= OBSTACLE_COST) continue;
						unsigned cost = pNeighbor->g_cost + neighborhood[i].baseCost;
						if(cost < minCost) {
							minCost = cost;
							pBack = pNeighbor;
						}
					}
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
					}
				} else painter.drawRect(QRectF(x - 0.25, y - 0.25, 0.5, 0.5));
				pCell++;

			}
			pCell += mapWidth() - (xEnd - xStart);
		}
	}
}

QString DStarLitePlanner::cellDetails(const QPoint &pos) {
	if(cells && pos.x() >= 0 && pos.x() < mapWidth() && pos.y() >= 0 && pos.y() < mapHeight()) {
		const Cell *pCell = cells + pos.y() * mapWidth() + pos.x();
		return QString().sprintf("Cell x = %d, y = %d%s\n - g_cost = %u\n - rhs = %u\n - key = (%u, %u)\n - heapIndex = %u",			
								 pCell->x, pCell->y, pCell->blocked ? " (Blocked)" : "",
								 pCell->g_cost, pCell->rhs, pCell->key.k1, pCell->key.k2,
								 pCell->heapIndex);
	}
	return QString();
}

void DStarLitePlanner::saveState(const QString &filename) const {
	if(!cells) return;
	
	QFile file(filename);
	if(file.open(QIODevice::WriteOnly)) {
		file.write((const char *)cells, sizeof(Cell) * mapWidth() * mapHeight());		
		file.close();		
	} else printf("Cannot save state to \"%s\". Error opening file.\n", qPrintable(filename));
}

QImage DStarLitePlanner::map() const {
	QImage map(mapSize(), QImage::Format_Indexed8);
	const Cell *pCell = cells;
	for(int y = 0; y < mapHeight(); y++) {
		unsigned char *dest = map.scanLine(y);
		for(int x = 0; x < mapWidth(); x++) {
			*dest++ = pCell->blocked ? 255 : 0;
			pCell++;
		}
	}
	return map;
}

void DStarLitePlanner::loadMapFromState(const QString &filename) {
	QFile file(filename);
	if(file.open(QIODevice::ReadOnly)) {
		if(file.size() == (sizeof(Cell) * mapWidth() * mapHeight())) {
			Cell *newCells = new (std::nothrow) Cell[mapWidth() * mapHeight()];
			
			if(newCells) {
				file.read((char *)newCells, sizeof(Cell) * mapWidth() * mapHeight());
				QImage map(mapSize(), QImage::Format_Indexed8);
				const Cell *pCell = newCells;
				for(int y = 0; y < map.height(); y++) {
					unsigned char *dest = map.scanLine(y);
					for(int x = 0; x < map.width(); x++) {
						*dest++ = pCell->blocked ? 255 : 0;
						pCell++;
					}
				}
				emit(mapChanged(map));				
				updateMap(map, map.rect());				

				//doCalculatePath(0, 81750);				
				// Inform GUI for redrawing
				emit dataChanged();

				
			} else printf("Cannot load map: out of memory\n");		
		} else printf("cannot load map state from \"%s\". File size mismatch.\n", qPrintable(filename));		
		file.close();
	} else printf("Cannot load map from \"%s\". Error opening file.\n", qPrintable(filename));	
}

void DStarLitePlanner::loadState(const QString &filename) {
	QFile file(filename);
	if(file.open(QIODevice::ReadOnly)) {
		if(file.size() == (sizeof(Cell) * mapWidth() * mapHeight())) {
			saveStateCounter = -1;
			
			file.read((char *)cells, sizeof(Cell) * mapWidth() * mapHeight());
			Cell *pCell = cells;
			const Cell *pCellEnd = cells + mapWidth() * mapHeight();
			openListLength = 0;			
			k_m = 0; // k_m is expected to be zero -> if required it can be recunstructed heuristically but a better option would be to include it into the state snapshot
			while(pCell != pCellEnd) {
				if(pCell->heapIndex > 0) {
					openHeap[pCell->heapIndex] = pCell;
					if(pCell->heapIndex > openListLength) openListLength = pCell->heapIndex;
				}
				pCell++;
			}
			
			if(listMap.size() != mapSize()) {
				listMap = QImage(mapSize(), QImage::Format_Indexed8);
				listMap.setColorTable(QVector<QRgb>() << qRgba(0, 0, 0, 0) << qRgba(255, 255, 0, 192) << qRgba(255, 128, 0, 192) << qRgba(0, 255, 255, 192) << qRgba(255, 0, 255, 192) << qRgb(0, 200, 0));
			}
			listMap.fill(0);		
			
			doDebugAndPathExtract(true);
			emit(mapChanged(map()));
			
		} else printf("cannot load state from \"%s\". File size mismatch.\n", qPrintable(filename));		
		file.close();		
	} else printf("Cannot load state from \"%s\". Error opening file.\n", qPrintable(filename));
}

void DStarLitePlanner::loadState() {
	loadState("dumps/dstarlite00000.bin");	
}
void DStarLitePlanner::loadMapState() {
	loadMapFromState("dumps/dstarlite00001.bin");
}


