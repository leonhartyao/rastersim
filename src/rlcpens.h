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

#ifndef RLCPENS_H
#define RLCPENS_H

#include "visualizationwidget.h"

namespace RLCPens {
	class SquarePen: public VisualizationWidget::RLCPen {
	public:	SquarePen(unsigned size = 1);
	};
	class CirclePen: public VisualizationWidget::RLCPen {
	public: CirclePen(unsigned size = 1);
	};
	class DiamondPen: public VisualizationWidget::RLCPen {
	public:	DiamondPen(unsigned size = 1);
	};
	class BarPen: public VisualizationWidget::RLCPen {
	public:
		BarPen(unsigned size = 1, Qt::Orientation orientation = Qt::Vertical);
	};	
}

#endif // RLCPENS_H
