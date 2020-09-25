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

#include "astarplanner.h"
#include <cstdio>
#include <new>
#include <QPainter>

AStarPlanner::AStarPlanner(QObject *parent):
	AbstractPlanner(parent),
	rasterElements(NULL),
	openList(NULL),
	visitedLayer(NULL)
{

}

AStarPlanner::~AStarPlanner() {
	freeMemory();
}
	
void AStarPlanner::freeMemory() {
	// free path planner memory 
	if(rasterElements) delete[] rasterElements;
	if(openList) delete[] openList;
}

void AStarPlanner::initMap(const QImage &map, const QRect &) {
	freeMemory(); // free old memory
	
	// allocate A* memory according to the image's dimensions
	rasterElements = new (std::nothrow) RasterElement[map.width() * map.height()];
	openList = new (std::nothrow) RasterElement *[map.width() * map.height() + 1];
		
	if(!rasterElements || !openList){
		printf("Could not allocate path planner memory\n");
		return;
	}
		
	RasterElement *pRE = rasterElements;	
	unsigned width = map.width();
	unsigned height = map.height();
	for(unsigned y = 0; y < height; y++) {
		const unsigned char *pCost = (const unsigned char *)map.scanLine(y);
		for(unsigned x = 0; x < width; x++) {
			pRE->x = x;
			pRE->y = y;
			pRE->list = (*pCost++ > 0) ? List_Unwalkable : List_None;
			pRE++;			
		}
	}
}

void AStarPlanner::drawDebugLayer(QPainter &painter, const DebugLayer *layer, const QRect &, qreal) {
	if(layer == visitedLayer) {
		painter.drawImage(QPointF(-0.5, -0.5), visitedMap);
	}
}

void AStarPlanner::calculatePath(InputUpdates) {	
	if(!openList || !rasterElements) {
		setError("Planner memory allocation error");
		return;
	}
	
	// some preparations...
	int width = mapWidth();
	int height = mapHeight();
	if(visitedMap.size() != mapSize()) {
		visitedMap = QImage(mapSize(), QImage::Format_Indexed8);
		visitedMap.setColorTable(QVector<QRgb>() << qRgba(0, 0, 0, 0) << qRgba(0, 255, 255, 128));
	}
	visitedMap.fill(0);
#define ADD_TO_VISITED_MAP(re)	visitedMap.setPixel((re)->x, (re)->y, 1);
		
	QPoint goalPos = this->goalPos().toPoint();
	QPoint startPos = this->startPos().toPoint();
	RasterElement *pStart = rasterElements + startPos.y() * width + startPos.x();
	RasterElement *pGoal = rasterElements + goalPos.y() * width + goalPos.x();
		
	// check validity of start & goal
	if(pGoal->list == List_Unwalkable) {
		setError("Goal position blocked");
		return;
	}	
	if(pStart->list == List_Unwalkable) {
		setError("Start position blocked");
		return;
	}

	// A* initialization
	RasterElement *pEnd = rasterElements + width * height - 1;
	RasterElement *pRE = rasterElements;	
	// clear open/closed lists
	for(int i = 0; i < width * height; i++){
		if(pRE->list != List_Unwalkable){
			pRE->list = List_None;
#ifdef HIGHQUALITYPATHPLANNER			
			int diffX = pTarget->x - pRE->x;
			int diffY = pTarget->y - pRE->y;
			pRE->h_cost = 10 * (int)sqrt(diffX * diffX + diffY * diffY);
#endif
		}
		pRE++;
	}

	pRE = pStart;
	pRE->g_cost = 0;
	// Add the starting location to the open list of squares to be checked.
	int numberOfOpenListItems = 1;
	openList[1] = pRE;
	ADD_TO_VISITED_MAP(pRE)

	int neighbourhood_offsets[8];
	// arrange neighbourhood pixels in a way that diagonal pixels have an even index (this will simplifies a condition used later)
	neighbourhood_offsets[0] = -width - 1;
	neighbourhood_offsets[1] = -width;
	neighbourhood_offsets[2] = -width + 1;
	neighbourhood_offsets[3] = -1;
	neighbourhood_offsets[4] = +width - 1;
	neighbourhood_offsets[5] = +1;
	neighbourhood_offsets[6] = +width + 1;
	neighbourhood_offsets[7] = +width;
	int neighbourhood_dx[8] = { -1,  0,  1, -1, -1, 1, 1, 0 };
	int neighbourhood_dy[8] = { -1, -1, -1,  0,  1, 0, 1, 1 };

	Path path;
	
	// Do the following until a path is found or deemed nonexistent.
	while(true) {
		// If the open list is not empty, take the first cell off of the list.
		// This is the lowest F cost cell on the open list.
		if(numberOfOpenListItems != 0) {
			// Pop the first item off the open list.
			pRE = openList[1];
			pRE->list = List_Closed;

			//	Open List = Binary Heap: Delete this item from the open list
			//	Delete the top item in binary heap and reorder the heap, with the lowest F cost item rising to the top.
			if(--numberOfOpenListItems){
				openList[1] = openList[numberOfOpenListItems + 1];//move the last item in the heap up to slot #1
				int v = 1;
	
				//	Repeat the following until the new item in slot #1 sinks to its proper spot in the heap.
				while(1){
					int u = v;	
					if (((u << 1) + 1) <= numberOfOpenListItems){ //if both children exist
					 	// Check if the F cost of the parent is greater than each child.
						// Select the lowest of the two children.
						if(openList[v]->f_cost >= openList[u << 1]->f_cost) v = u << 1;
						if(openList[v]->f_cost >= openList[(u << 1) + 1]->f_cost) v = (u << 1) + 1;								
					}else{
						if((u << 1) <= numberOfOpenListItems){ //if only child #1 exists
					 		// Check if the F cost of the parent is greater than child #1	
							if(openList[u]->f_cost >= openList[u << 1]->f_cost) v = (u << 1);
						}
					}
				
					if (u != v){ // if parent's F is > one of its children, swap them
						RasterElement *temp = openList[u];
						openList[u] = openList[v];
						openList[u]->openListIndex = u;
						openList[v] = temp;			
						openList[v]->openListIndex = v;
					} else break; //otherwise, exit loop
				}
			}
		
			// Check the adjacent cells. (Its "children" -- these path children
			// are similar, conceptually, to the binary heap children mentioned
			// above, but don't confuse them. They are different. Path children
			// are portrayed in Demo 1 with grey pointers pointing toward
			// their parents.) Add these adjacent child squares to the open list
			// for later consideration if appropriate (see various if statements
			// below).

			for(int neighbourhood_index = 0; neighbourhood_index < 8; neighbourhood_index++){
				int x = pRE->x + neighbourhood_dx[neighbourhood_index];
				int y = pRE->y + neighbourhood_dy[neighbourhood_index];
				if((unsigned)x >= (unsigned)width || (unsigned)y >= (unsigned)height) continue;
				
				//	If not off the map (do this first to avoid array out-of-bounds errors)
				RasterElement *pNeighbour = pRE + neighbourhood_offsets[neighbourhood_index];
				if((pNeighbour >= rasterElements) && (pNeighbour < pEnd)){
					//	If not already on the closed list (items on the closed list have
					//	already been considered and can now be ignored).			
					if(pNeighbour->list != List_Closed){ 
						// If not a wall/obstacle square.
						if(pNeighbour->list != List_Unwalkable){ 
							//	If not already on the open list, add it to the open list.			
							if(pNeighbour->list != List_Open){	
								//Create a new open list item in the binary heap.
								int m = numberOfOpenListItems + 1;
								openList[m] = pNeighbour;
								pNeighbour->openListIndex = m;
								
								// Figure out its G cost
								pNeighbour->g_cost = pRE->g_cost;
								if(neighbourhood_index & 0x01) // non-diagonal neighbour									
									pNeighbour->g_cost += 10;							
								else	
									pNeighbour->g_cost += 14; // diagonal member				
								// Figure out its H and F costs and parent 
#ifndef HIGHQUALITYPATHPLANNER
								int h_cost = 10 * (abs(pNeighbour->x - pGoal->x) + abs(pNeighbour->y - pGoal->y));
#else
								int h_cost = pNeighbour->h_cost;
#endif			
								pNeighbour->f_cost = pNeighbour->g_cost + h_cost;
								pNeighbour->parent = pRE; 
						
								// Move the new open list item to the proper place in the binary heap.
								// Starting at the bottom, successively compare to parent items,
								// swapping as needed until the item finds its place in the heap
								// or bubbles all the way to the top (if it has the lowest F cost).
								while(m != 1){ // While item hasn't bubbled to the top (m=1)	
									// Check if child's F cost is < parent's F cost. If so, swap them.	
									int m_half = m >> 1;
									if (openList[m]->f_cost <= openList[m_half]->f_cost){
										RasterElement *temp = openList[m_half];
										//temp->openListIndex = m;
										openList[m_half] = openList[m];
										openList[m_half]->openListIndex = m_half;
										openList[m] = temp;
										openList[m]->openListIndex = m;
										m = m_half;
									} else break;
								}
								numberOfOpenListItems++;
	
								//Change whichList to show that the new item is on the open list.
								pNeighbour->list = List_Open;
								ADD_TO_VISITED_MAP(pNeighbour)
							} else {
								// If adjacent cell is already on the open list, check to see if this 
								// path to that cell from the starting location is a better one. 
								// If so, change the parent of the cell and its G and F costs.	
								
								// Figure out the G cost of this possible new path
							
								int tempGcost = pRE->g_cost;
								if(neighbourhood_index & 0x01) // non-diagonal neighbour									
									tempGcost += 10;							
								else	
									tempGcost += 14; // diagonal member				
			
								//If this path is shorter (G cost is lower) then change
								//the parent cell, G cost and F cost. 		
								if(tempGcost < pNeighbour->g_cost){ //if G cost is less,
#ifdef HIGHQUALITYPATHPLANNER
									int h_cost = pNeighbour->h_cost;
#else
									int h_cost = pNeighbour->f_cost - pNeighbour->g_cost;
#endif									
									pNeighbour->g_cost = tempGcost;
									pNeighbour->f_cost = tempGcost + h_cost;
									pNeighbour->parent = pRE;
																		
									//See if changing the F score bubbles the item up from it's current location in the heap
									int m = pNeighbour->openListIndex;
									
									while(m != 1){ //While item hasn't bubbled to the top (m=1)	
										// Check if child is < parent. If so, swap them.	
										int m_half = m >> 1;
										if(openList[m]->f_cost < openList[m_half]->f_cost){
											RasterElement *temp = openList[m_half];
											openList[m_half] = openList[m];
											openList[m_half]->openListIndex = m_half;
											openList[m] = temp;
											openList[m]->openListIndex = m;
											m = m_half;
										} else break;
									} 
								}
							}
						} 
					}
				}
			}
	
		} else {
			setError("No Path found");
			break;
		}
	
		//If target is added to open list then path has been found.
		if(pGoal->list == List_Closed){
			// Path found, extract path data into QVector and return that
			// 1st step: examine path length
			int pathLength = 0;
			pRE = pGoal;
			while(1){
				pathLength++;
				if(pRE == pStart) break;
				else pRE = pRE->parent;	
			}	
			
			// 2nd step: store path points	
			path.resize(pathLength);
			pRE = pGoal;
			int segmentIndex = pathLength - 1;
			while(1) {				
				path[segmentIndex--] = QPointF(pRE->x, pRE->y);
				if(pRE == pStart) break;
				else pRE = pRE->parent;
			}
			
			break;
		}
	}
	
	if(!visitedLayer) addDebugLayer(visitedLayer = new DebugLayer(tr("Show visited cells")));
		
	setPath(path);
}
