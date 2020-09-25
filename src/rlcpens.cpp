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

#include "rlcpens.h"


namespace RLCPens {
	SquarePen::SquarePen(unsigned size) {
		Runs runs;
		int sz = qMax(1U, size);
		int start = -(sz / 2);
		int y = start;
		for(int i = 0; i < sz; i++, y++) runs.push_back(Run(y, start, sz, 255));
		setRuns(runs);
	}
	
	CirclePen::CirclePen(unsigned size) {
		if(size < 1) size = 1;
		Runs runs;
		int radius = size >> 1;
		
		if(radius >= 1) {
			unsigned twoRadSquare = (radius * radius);
			int xChange = twoRadSquare * (1 - (radius << 1));
			int yChange = twoRadSquare;
			twoRadSquare <<= 1;
			int ellipseError = 0;
			int xStop = twoRadSquare * radius;
			int yStop = 0; 

			int y0u = 0;
			int y0d = (size & 1) ? 0 : 1;
			int y1u = y0d - radius;
			int y1d = radius;
			int x0 = -radius;
			int x1 = -y0d;
			int w0 = size;
			int w1 = 1 + y0d;
			
			while(xStop >= yStop){
				runs.push_back(Run(y0u, x0, w0, 255));
				if(y0u != y0d) runs.push_back(Run(y0d, x0, w0, 255));
										
				yStop += twoRadSquare;
				ellipseError += yChange;
				yChange += twoRadSquare;
				if((ellipseError << 1) + xChange > 0){
					if(y1d != y0d) runs.push_back(Run(y1d, x1, w1, 255));
					if(y1u != y0u) runs.push_back(Run(y1u, x1, w1, 255));
					x0++;
					w0 -= 2;
					y1d--;
					y1u++;
					xStop -= twoRadSquare;
					ellipseError += xChange;
					xChange += twoRadSquare;
				}
				if(y0d >= y1d) break;
				y0u--;
				y0d++;
				x1--;
				w1 += 2;
			}							
		} else runs.push_back(Run(0, 0, 1, 255));
		setRuns(runs);
	}

	DiamondPen::DiamondPen(unsigned size) {
		Runs runs;
		int sz = qMax(1U, size);
		int x = -(sz / 2);
		int yu = 0;
		int yd = (sz & 1) ? 0 : 1;
		while(sz > 0) {
			runs.push_back(Run(yu, x, sz, 255));
			if(yu != yd) runs.push_back(Run(yd, x, sz, 255));
			yu--;
			yd++;
			x++;
			sz -= 2;
		}
		setRuns(runs);
	}

	BarPen::BarPen(unsigned size, Qt::Orientation orientation) {
		Runs runs;
		int sz = qMax(1U, size);
		if(orientation == Qt::Vertical) {
			int y = -(sz / 2);
			for(int i = 0; i < sz; i++, y++) runs.push_back(Run(y, 0, 1, 255));
		} else runs.push_back(Run(0, -(sz / 2), sz, 255));
		setRuns(runs);
	}
}

