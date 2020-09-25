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

#include "visualizationwidget.h"
#include <QPainter>
#include <QImage>
#include <cstdio>
#include "abstractplanner.h"
#include <QBitmap>

VisualizationWidget::VisualizationWidget(QWidget *parent):
	ZoomableWidget(parent),
	_map(NULL, NULL), _planner(NULL),
	_start(Pose2D::invalid()), _goal(Pose2D::invalid()),
	mouseObject(Mouse_Nothing),
	_layerModel(NULL),
	_activeTool(Tool_None), _tool(Tool_Pointer), _toolCost(255),
	penCursor(QBitmap(":images/pen_cursor.bmp"), QBitmap(":images/pen_cursor_mask.bmp"), 0, 19)		
{
	layers.push_back(Layer(Layer_Map));
	layers.push_back(Layer(Layer_Path));
	layers.push_back(Layer(Layer_StartGoal));
	_layerModel = new LayerModel(this);
	setBackground(QColor(96, 96, 96));
	
}

void VisualizationWidget::setMap(const QImage &map) {
	if(map.format() != QImage::Format_Indexed8) return;
	if(!map.isNull()) {
		_map = map;
		// adapt color table
		// check for correct color table		
		QVector<QRgb> table(256);
		for(unsigned i = 0; i < 256; i++) table[255 - i] = qRgb(i, i, i);
		if(_map.colorTable() != table) 
			_map.setColorTable(table); // check for coincidence to prevent deep copy
		
		setWorld(_map.size(), QPointF(-0.5, -0.5));
		
		QRect rc(QPoint(0, 0), map.size());
		if(_start.isValid()) {
			if(!rc.contains(_start.pos().toPoint())) _start = Pose2D::invalid();
		}
		if(_goal.isValid()) {
			if(!rc.contains(_goal.pos().toPoint())) _goal = Pose2D::invalid();
		}
		
		if(_planner) {
			_planner->setMap(_map);
			if(_start.isValid()) _planner->setStart(_start);
			if(_goal.isValid()) _planner->setGoal(_goal);
		}		
	} else {
		clear();
		if(_planner) _planner->setMap(_map);
	}
}

void VisualizationWidget::handleMapChangeFromPlanner(const QImage map) {
	if(map.size() == _map.size()) {
		QVector<QRgb> colorTable = _map.colorTable();
		_map = map;
		_map.setColorTable(colorTable);
		updateContent();
	}
}

void VisualizationWidget::setPlanner(AbstractPlanner *planner) {
	if(_planner) {
		// remove all items from the planner in the model
		_planner->disconnect(this);
		for(int i = layers.size() - 1; i >= 0; i--) {
			if(layers[i].type == LayerType_Planner) {
				_layerModel->beginRemoveRows(QModelIndex(), i, i);
				layers.removeAt(i);
				_layerModel->endRemoveRows();
			}
		}
	}
	
	_planner = planner;
	if(_planner) {
		if(!_planner->debugLayers().isEmpty()) {
			int idx = -1;
			for(int i = 0; i < layers.size(); i++) if(layers[i].type == LayerType_Internal && layers[i].internalLayer == Layer_Map) idx = i;
			idx++;
			_layerModel->beginInsertRows(QModelIndex(), idx, idx + _planner->debugLayers().size() - 1);
			for(int i = 0; i < _planner->debugLayers().size(); i++) layers.insert(idx++, _planner->debugLayers()[i]);
			_layerModel->endInsertRows();
		}
		connect(_planner, SIGNAL(dataChanged()), this, SLOT(updatePlannerData()));
		connect(_planner, SIGNAL(configChanged(AbstractPlanner::ConfigElement, AbstractPlanner::ConfigChange, int)), 
				this, SLOT(handlePlannerConfigChanged(AbstractPlanner::ConfigElement, AbstractPlanner::ConfigChange, int)));
		connect(_planner, SIGNAL(mapChanged(const QImage)), this, SLOT(handleMapChangeFromPlanner(const QImage)));
		
		_planner->setMap(_map);
		if(_start.isValid()) _planner->setStart(_start);
		if(_goal.isValid()) _planner->setGoal(_goal);
	}
	updateContent();
}

void VisualizationWidget::updatePlannerData() {
	updateContent();
}
void VisualizationWidget::handlePlannerConfigChanged(AbstractPlanner::ConfigElement element, AbstractPlanner::ConfigChange type, int index) {
	if(element != AbstractPlanner::Element_DebugLayer) return;
	
	if(type == AbstractPlanner::Change_Add) {
		int idx = -1;
		int idxMap = -1;
		for(int i = 0; i < layers.size(); i++) {
			if(layers[i].type == LayerType_Planner) idx = i;
			if(layers[i].type == LayerType_Internal && layers[i].internalLayer == Layer_Map) idxMap = i;
		}
		if(idx == -1) idx = idxMap;
		idx++; // insert after the element found above
		_layerModel->beginInsertRows(QModelIndex(), idx, idx);
		layers.insert(idx, Layer(_planner->debugLayers()[index]));
		_layerModel->endInsertRows();
		if(layers[idx].visible) updateContent();		
	} else {
		// find layer index for debugLayer
		const AbstractPlanner::DebugLayer *debugLayer = _planner->debugLayers().at(index);
		for(int i = 0; i < layers.size(); i++) {
			if(layers[i].type == LayerType_Planner && layers[i].plannerDebugLayer == debugLayer) {
				bool wasVisible = layers[i].visible;
				if(type == AbstractPlanner::Change_Remove) {
					_layerModel->beginRemoveRows(QModelIndex(), i, i);
					layers.removeAt(i);
					_layerModel->endRemoveRows();					
				} else _layerModel->emitDataChanged(_layerModel->index(i));
				if(wasVisible) updateContent();
				break;
			}
		}
	}
}

void VisualizationWidget::paintContent(QPainter &painter) {
	for(int i = 0; i < layers.size(); i++) {
		const Layer &l = layers[i];
		if(l.visible) {
			painter.setOpacity(1.0);
			
			switch(l.type){
			case LayerType_Internal:
				switch(l.internalLayer) {
				case Layer_Map:
					// draw the map...
					painter.drawImage(QPointF(-0.5, -0.5), _map);					
					break;
					
				case Layer_Path:
					// paint path
					if(_planner) {
						const Path &path = _planner->path();
						if(path.count() > 0) {
							QPen pathPen(QPen(QColor(255, 0, 0), 3));
							pathPen.setCosmetic(true);
							painter.setPen(pathPen);
							QPointF last = path[0];
							for(int i = 1; i < path.count(); i++){				
								painter.drawLine(last, path[i]);
								last = path[i];
							}
						}
					}		
					break;
					
				case Layer_StartGoal:
					// paint start+goal
					if(_start.isValid()) {
						painter.setPen(QPen(Qt::black, 0));
						painter.setBrush(QColor(192, 192, 192, 192));
						painter.setOpacity(1.0);
						painter.drawEllipse(_start.pos(), roboRadius, roboRadius);			
						painter.setPen(QPen(QColor(128, 0, 0), 0));
						painter.drawLine(_start.pos(), _start.pos() + roboRadius * QPointF(cos(_start.angle()), sin(_start.angle())));
					}
					if(_goal.isValid()) {
						painter.setPen(QPen(Qt::black, 0));
						painter.setBrush(QColor(64, 192, 64));
						painter.setOpacity(0.5);
						painter.drawEllipse(_goal.pos(), roboRadius, roboRadius);			
						painter.setPen(QPen(QColor(128, 0, 0), 0));
						painter.drawLine(_goal.pos(), _goal.pos() + roboRadius * QPointF(cos(_goal.angle()), sin(_goal.angle())));			
					}
					break;
				}
				break;
			case LayerType_Planner:
				{
					qreal zoom = zoomFactor();
					AbstractPlanner::DebugLayer *debug = l.plannerDebugLayer;
					if(zoom >= debug->minimumZoomFactor() && zoom <= debug->maximumZoomFactor()) {
						QTransform t = painter.transform().inverted();
						QRectF area = t.mapRect(QRectF(rect())).normalized();
						area.setLeft((int)area.left());
						area.setTop((int)area.top());
						area.setWidth(ceil(area.width()));
						area.setHeight(ceil(area.height()));
						debug->draw(painter, area.toRect().intersected(_map.rect()), zoom);
					}
				}
				break;
			default: break;
			}
		}			
	}
	
	painter.setOpacity(1.0);	
	switch(_activeTool) {
	case Tool_Rect:
		painter.setBrush(QColor(255, 0, 255, 128));
		painter.setPen(Qt::NoPen);
		painter.drawRect(QRectF(toolBoundingRect).translated(-0.5, -0.5));
		break;
	case Tool_Line:
		painter.setPen(QPen(QColor(255, 0, 255, 128), (_pen.boundingRect().width() + _pen.boundingRect().height()) >> 1));
		painter.drawLine(QPointF(toolBoundingRect.topLeft()), QPointF(toolBoundingRect.bottomRight()) + QPointF(0.001, 0));
		break;
	case Tool_Pen:
	case Tool_Pointer:
	default:
		if(!toolBoundingRect.isNull()) {		
			painter.setBrush(Qt::NoBrush);
			painter.setPen(QPen(QColor(255, 0, 0), 0));
			painter.drawRect(QRectF(toolBoundingRect).translated(-0.5, -0.5));		
			painter.setPen(QPen(QColor(0, 255, 0), 0, Qt::DashLine));
			painter.drawRect(QRectF(toolBoundingRect).translated(-0.5, -0.5));		
		}
	}
}

void VisualizationWidget::paintOverlays(QPainter &painter, const QRect &area) {
	if(_planner) {
		if(_planner->path().isEmpty()) {
			painter.setPen(QPen(QColor(255, 0, 0)));
			painter.setFont(QFont("Verdana", 36));
			painter.drawText(area, Qt::AlignCenter, _planner->lastError());
		}
	}
}
 
void VisualizationWidget::setStart(const Pose2D &start) {
	_start = start;
	if(mouseObject == Mouse_Start) mouseObject = Mouse_Nothing;
	updateContent();
}
void VisualizationWidget::setGoal(const Pose2D &goal) {
	_goal = goal;
	if(mouseObject == Mouse_Goal) mouseObject = Mouse_Nothing;
	updateContent();	
}

void VisualizationWidget::worldMousePressEvent(const QPointF &pos, Qt::MouseButtons, Qt::MouseButton button) {
	if(_activeTool != Tool_None) return;	
	
	_activeTool = _tool;
	if(button != Qt::LeftButton && button != Qt::RightButton) {
		_activeTool = Tool_None;
		return;
	}
	
	mouseReleaseButton = button;
	mouseDownPos = QPointF(qBound(0.0, pos.x(), (qreal)(_map.width() - 1)), qBound(0.0, pos.y(), (qreal)(_map.height() - 1)));

	switch(_activeTool) {
	case Tool_Pointer:
		if(button == Qt::LeftButton) {
			if(_start.isValid()) mouseObject = Mouse_Goal;
			else mouseObject = Mouse_Start;
		} else if(button == Qt::RightButton) mouseObject = Mouse_Start;
		
		if(mouseObject != Mouse_Nothing) {
			if(mouseObject == Mouse_Start) _start = Pose2D(mouseDownPos, isnan(_start.angle()) ? 0.0 : _start.angle());
			else if(mouseObject == Mouse_Goal) _goal = Pose2D(mouseDownPos, isnan(_goal.angle()) ? 0.0 : _goal.angle());
			updateContent();
		}
		break;
		
	case Tool_Pen:
		{
			QPoint startPos = mouseDownPos.toPoint();
			toolBoundingRect = _pen.boundingRect().translated(startPos);
			addPoint(startPos.x(), startPos.y(), mouseReleaseButton != Qt::LeftButton);
			updateContent();
		}
		break;
	case Tool_Line:
	case Tool_Rect:
		toolBoundingRect = QRect(mouseDownPos.toPoint(), QSize(1, 1));
		updateContent();
		break;
	
	default: break;
	}
}
void VisualizationWidget::worldMouseMoveEvent(const QPointF &pos, Qt::MouseButtons) {
	if(_activeTool == Tool_None) return;
	
	QPointF mousePos = QPointF(qBound(0.0, pos.x(), (qreal)(_map.width() - 1)), qBound(0.0, pos.y(), (qreal)(_map.height() - 1)));
	
	switch(_activeTool) {
	case Tool_Pointer:
		if(mouseObject == Mouse_Start || mouseObject == Mouse_Goal) {
			QPointF delta = pos - mouseDownPos;
			if(delta.x() != 0.0 || delta.y() != 0.0) {
				if(mouseObject == Mouse_Start) _start.setAngle(atan2(delta.y(), delta.x()));
				else _goal.setAngle(atan2(delta.y(), delta.x()));
				updateContent();
			}			
		}
		break;
	case Tool_Line:
		toolBoundingRect.setBottomRight(mousePos.toPoint());
		updateContent();
		break;
	case Tool_Pen:
		{
			QPoint intPos = mousePos.toPoint();
			addLine(mouseDownPos.toPoint(), intPos, mouseReleaseButton != Qt::LeftButton);
			mouseDownPos = intPos;
		}
		updateContent();
		break;
		
	case Tool_Rect:
		{			
			QPoint p1 = mouseDownPos.toPoint();
			QPoint p2 = mousePos.toPoint();
			toolBoundingRect = QRect(qMin(p1.x(), p2.x()), qMin(p1.y(), p2.y()), abs(p1.x() - p2.x()) + 1, abs(p1.y() - p2.y()) + 1);
		}
		updateContent();		
		break;
	default: break;
	}
}
void VisualizationWidget::worldMouseReleaseEvent(const QPointF &, Qt::MouseButtons buttons, Qt::MouseButton) {
	if(!(buttons & mouseReleaseButton)) {
		switch(_activeTool) {
		case Tool_Pointer:
			if(mouseObject == Mouse_Start) {
				if(_planner) _planner->setStart(_start);
				emit startPoseChanged();
			} else if(mouseObject == Mouse_Goal) {
				if(_planner) _planner->setGoal(_goal);
				emit goalPoseChanged();	
			}
			mouseObject = Mouse_Nothing;
		
			break;
		
		case Tool_Line:
			addLine(toolBoundingRect.topLeft(), toolBoundingRect.bottomRight(), mouseReleaseButton != Qt::LeftButton);
			toolBoundingRect = _map.rect().intersected(toolBoundingRect);
			if(_planner) _planner->updateMap(_map, toolBoundingRect);
			updateContent();
			break;
		
		case Tool_Pen:
			toolBoundingRect = _map.rect().intersected(toolBoundingRect);
			if(_planner) _planner->updateMap(_map, toolBoundingRect);
			break;
		case Tool_Rect:
			addRect(toolBoundingRect, mouseReleaseButton == Qt::LeftButton ? _toolCost : 255 - _toolCost);
			if(_planner) _planner->updateMap(_map, toolBoundingRect);
			updateContent();
			break;
			
		default: break;
		}
	
		_activeTool = Tool_None;
	}
}

void VisualizationWidget::addPoint(int xCenter, int yCenter, bool invert) {
	int mapWidth = _map.width();
	int mapHeight = _map.height();
	foreach(const struct RLCPen::Run &run, _pen.runs()) {
		int y = yCenter + run.y;
		if(y < 0 || y >= mapHeight) continue;
		int x = xCenter + run.xStart;
		int w = run.xLength;
		if(x < 0) {
			w += x;
			x = 0;
		}
		if(x + w >= mapWidth) w -= x + w - mapWidth;
		if(w <= 0) continue;		
		
		memset(_map.scanLine(y) + x, invert ? 255 - run.cost : run.cost, w);		
	}	
}

void VisualizationWidget::addLine(const QPoint &p1, const QPoint &p2, bool invert) {
	// TODO: bresenham algorithm
	int x1 = p1.x();
	int y1 = p1.y();
	int x2 = p2.x();
	int y2 = p2.y();
	

	if(x1 > x2){
		// swap (x1, y1) <-> (x2, y2)
		int temp = x1; x1 = x2; x2 = temp;
		temp = y1; y1 = y2; y2 = temp;
	}

	int dx = x2 - x1;
	int dy = y2 - y1;
	if(dy >= 0){		
		if(dx > dy){ 
			// 1st octant
			int err = dx;
			dy <<= 1; dx <<= 1;
			while(x1 <= x2){
				addPoint(x1, y1, invert);
				err -= dy;
				if(err < 0) {
					y1++;
					err += dx;
				}
				x1++;
			}
		}else{ 
			// 2nd octant
			int err = dy;
			dy <<= 1; dx <<= 1;
			while(y1 <= y2){
				addPoint(x1, y1, invert);
				err -= dx;
				if(err < 0){
					x1++;
					err += dy;
				}
				y1++;
			}
		}
	} else {
		dy = -dy; // make dy positive
		if(dx > dy){ 
			// 8th octant
			int err = dx;
			dy <<= 1; dx <<= 1;
			while(x1 <= x2){
				addPoint(x1, y1, invert);
				err -= dy;
				if(err < 0){
					y1--;
					err += dx;
				}
				x1++;
			}
		}else{ 
			// 7th octant
			int err = dy;
			dy <<= 1; dx <<= 1;
			while(y1 >= y2){
				addPoint(x1, y1, invert);
				err -= dx;
				if(err < 0){
					x1++;
					err += dy;
				}
				y1--;
			}
		}	
	}
	
	QRect rc = _pen.boundingRect();	
	toolBoundingRect = toolBoundingRect.united(
							QRect(qMin(p1.x(), p2.x()), qMin(p1.y(), p2.y()), 1 + abs(p1.x() - p2.x()), 1 + abs(p1.y() - p2.y()))
								.adjusted(rc.left(), rc.top(), rc.right(), rc.bottom()));
}

void VisualizationWidget::addRect(const QRect &rc, unsigned char cost) {
	// Precondition: we have a map and the rect is completely contained within this map and the rect is normalized
	if(!rc.isValid()) return;
	
	unsigned x = rc.left();
	unsigned w = rc.width();
	for(int y = rc.top(); y <= rc.bottom(); y++) memset(_map.scanLine(y) + x, cost, w);
}

void VisualizationWidget::setPen(const RLCPen &pen) {
	_pen = pen;
}

void VisualizationWidget::setTool(Tool tool) {
	_tool = tool;
	switch(tool) {
	case Tool_Pen:
		setCursor(penCursor); 
		break;
	case Tool_Line:
	case Tool_Rect:
		setCursor(Qt::CrossCursor);
		break;
	case Tool_Pointer: 
	default: unsetCursor();
	}
}

void VisualizationWidget::setToolCost(int cost) {
	if(cost < 0) cost = 0; else if(cost > 255) cost = 255;
	_toolCost = cost;
}

////////////////////////////////////////////////////////////////////////////////
// LayerModel
////////////////////////////////////////////////////////////////////////////////

VisualizationWidget::LayerModel::LayerModel(VisualizationWidget *parent): QAbstractListModel(parent), vis(parent) { }

int VisualizationWidget::LayerModel::rowCount(const QModelIndex &) const {
	return vis->layers.size();
}
QVariant VisualizationWidget::LayerModel::data(const QModelIndex &index, int role) const {
	int idx = index.row();
	if(idx >= 0 && idx < vis->layers.size()) {
		const Layer &l = vis->layers[idx];
		if(role == Qt::DisplayRole) {
			switch(l.type) {
			case LayerType_Internal:
				switch(l.internalLayer) {
				case Layer_Map: return tr("Map");
				case Layer_Path: return tr("Path");
				case Layer_StartGoal: return tr("Start & Goal");
				}	
				break;
			case LayerType_Planner:	return l.plannerDebugLayer->name();
			default: break;
			}
			return "???";
		} else if(role == Qt::CheckStateRole) {
			return l.visible ? Qt::Checked : Qt::Unchecked;
		}
	}
	return QVariant();
}
QVariant VisualizationWidget::LayerModel::headerData(int section, Qt::Orientation orientation, int role) const {
	if(role == Qt::DisplayRole && orientation == Qt::Horizontal && section == 0) return "Visualization Layer";
	return QVariant();
}

bool VisualizationWidget::LayerModel::setData(const QModelIndex &index, const QVariant &value, int role) {
	if (!index.isValid()) return false;
	if(role == Qt::CheckStateRole) {
		int idx = index.row();
		if(idx >= 0 && idx < vis->layers.size()) {
			vis->layers[idx].visible = (static_cast<Qt::CheckState>(value.toUInt()) == Qt::Checked);
			emit dataChanged(index, index);
			vis->updateContent();
			return true;
		}		
	}
	return QAbstractItemModel::setData(index, value, role);
}
Qt::ItemFlags VisualizationWidget::LayerModel::flags(const QModelIndex &index) const {
	return QAbstractListModel::flags(index) | Qt::ItemIsUserCheckable;
}

////////////////////////////////////////////////////////////////////////////////
// class RLCPen
////////////////////////////////////////////////////////////////////////////////

void VisualizationWidget::RLCPen::setRuns(const Runs &runs) {
	_runs = runs;
	_image = QImage();
	_boundingRect = QRect();
}

QImage VisualizationWidget::RLCPen::toImage() const {
	if(_image.isNull()) {
		QRect rc = boundingRect();
		_image = QImage(rc.size(), QImage::Format_Indexed8);
		QVector<QRgb> colorTable(256);
		for(int i = 0; i < 256; i++) colorTable[255 - i] = qRgb(i, i, i);
		_image.setColorTable(colorTable);
		_image.fill(0);
		foreach(const struct Run &run, _runs) memset(_image.scanLine(run.y - rc.top()) + run.xStart - rc.left(), run.cost, run.xLength);
	}
	return _image;
}
QRect VisualizationWidget::RLCPen::boundingRect() const {
	if(_boundingRect.isNull()) {
		int left = INT_MAX;
		int right = INT_MIN;
		int top = INT_MAX;
		int bottom = INT_MIN;
		foreach(const struct Run &run, _runs) {
			if(run.y < top) top = run.y;
			if(run.y > bottom) bottom = run.y;
			if(run.xStart < left) left = run.xStart;
			if(run.xEnd() > right) right = run.xEnd();
		}
		_boundingRect = QRect(QPoint(left, top), QPoint(right, bottom));
	} 
	return _boundingRect;
}
void VisualizationWidget::RLCPen::setCost(unsigned char cost) {
	for(int i = 0; i < _runs.size(); i++) _runs[i].cost = cost;
	_image = QImage();
}
