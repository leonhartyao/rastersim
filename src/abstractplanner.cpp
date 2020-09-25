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

#include "abstractplanner.h"
#include <QRectF>
#include <QTime>
#include <cstdio>

AbstractPlanner::AbstractPlanner(QObject *parent): 
	QObject(parent),
	_start(Pose2D::invalid()), _goal(Pose2D::invalid()),
	_calcTimeMs(-1), 
	inDestructor(false),
	accumulatedInputUpdates(NoInputUpdates)
{
	
}

AbstractPlanner::~AbstractPlanner() {	
	inDestructor = true;
	while(!_debugLayers.empty()) delete _debugLayers.takeLast();
}

void AbstractPlanner::setStart(const Pose2D &start) {	
	if(!start.isValid()) {
		_start = start; 
		_path.clear();
		emit dataChanged();
		return;
	}
	if(QRect(QPoint(0, 0), _mapSize).contains(start.pos().toPoint())) {
		_start = start;
		accumulatedInputUpdates |= UpdatedStart;
		callPlanner();		
	}
}

void AbstractPlanner::setStart(const QPointF &start) {
	setStart(Pose2D(start, isnan(_start.angle()) ? 0.0 : _start.angle()));
}
void AbstractPlanner::setGoal(const Pose2D &goal) {
	if(!goal.isValid()) {
		_goal = goal;
		_path.clear();
		emit dataChanged();
		return;
	}
	if(QRect(QPoint(0, 0), _mapSize).contains(goal.pos().toPoint())) {
		_goal = goal;
		accumulatedInputUpdates |= UpdatedGoal;
		callPlanner();
	}	
}
void AbstractPlanner::setGoal(const QPointF &goal) {
	setGoal(Pose2D(goal, isnan(_goal.angle()) ? 0.0 : _goal.angle()));
}
void AbstractPlanner::setStartGoal(const Pose2D &start, const Pose2D &goal) {
	bool startValid = start.isValid();
	bool goalValid = goal.isValid();
	if(startValid && goalValid) {
		QRect rc(QPoint(0, 0), _mapSize);
		if(!rc.contains(start.pos().toPoint())) return;
		if(!rc.contains(goal.pos().toPoint())) return;
		
		_start = start;
		_goal = goal;
		accumulatedInputUpdates |= UpdatedStart | UpdatedGoal;
		callPlanner();		
	} else {
		if(!startValid) {
			setStart(start);
			setGoal(goal);
		} else {
			setGoal(goal);
			setStart(start);
		}		
	}	
}
void AbstractPlanner::setStartGoal(const QPointF &start, const QPointF &goal) {
	setStartGoal(Pose2D(start, isnan(_start.angle()) ? 0.0 : _start.angle()),
				 Pose2D(goal, isnan(_goal.angle()) ? 0.0: _goal.angle()));
}

void AbstractPlanner::setMap(const QImage &mapData) {
	if(mapData.format() != QImage::Format_Indexed8) return;
	
	_start = Pose2D::invalid();
	_goal = Pose2D::invalid();
	
	_path.clear();
	_mapSize = mapData.size();
	initMap(mapData, QRect());
	accumulatedInputUpdates = NewMap;
		
	emit dataChanged();
}

void AbstractPlanner::updateMap(const QImage &mapData, const QRect &updateRegion) {
	if(mapData.format() != QImage::Format_Indexed8) return;
	
	if(mapData.size() == _mapSize) {
		if(updateRegion.isEmpty() || !mapData.rect().contains(updateRegion)) return;
		
		initMap(mapData, updateRegion);
		if(!(accumulatedInputUpdates & NewMap)) accumulatedInputUpdates |= UpdatedMap;
		callPlanner();
	} else setMap(mapData);	
}

void AbstractPlanner::setPath(const Path &path) {
	_path = path;
	if(!_path.empty()) _lastError.clear();
}
void AbstractPlanner::setError(const QString &str) {
	_lastError = str;
}
void AbstractPlanner::setError(const char *format, ...) {
	char *outStr = NULL; 
	va_list args; va_start(args, format);	
	int ret __attribute__((unused)) = vasprintf(&outStr, format, args);
	va_end(args);
	setError(QString(outStr));
	free(outStr);
}

void AbstractPlanner::callPlanner() {
	if(!_mapSize.isEmpty() && _start.isValid() && _goal.isValid()) {	
		_lastError.clear();
		_path.clear();
		QTime time;
		time.start();				
		calculatePath(accumulatedInputUpdates);
		_calcTimeMs = time.elapsed();
		accumulatedInputUpdates = NoInputUpdates;
		
		if(_path.empty() && _lastError.isEmpty()) _lastError = "No Path set";
		emit dataChanged();
	}	
}

void AbstractPlanner::addDebugLayer(DebugLayer *layer, DebugLayer *before) {
	addDebugLayer(layer, _debugLayers.indexOf(before));
}	
void AbstractPlanner::addDebugLayer(DebugLayer *layer, int before) {
	if(!layer) return;
	if(layer->_planner) return;
	
	if(before < 0 || before >= _debugLayers.size()) before = _debugLayers.size();
	_debugLayers.insert(before, layer);
	layer->_planner = this;
	emit configChanged(Element_DebugLayer, Change_Add, before);
}
void AbstractPlanner::removeDebugLayer(DebugLayer *layer) {
	int idx = _debugLayers.indexOf(layer);
	if(idx >= 0 && idx < _debugLayers.size()) {
		if(!inDestructor) emit configChanged(Element_DebugLayer, Change_Remove, idx);
		layer->_planner = NULL;
		_debugLayers.removeAt(idx);
	}
}

void AbstractPlanner::addAction(QAction *action) {
	_actions.push_back(action);
}

////////////////////////////////////////////////////////////////////////////////
// class DebugLayer
////////////////////////////////////////////////////////////////////////////////

AbstractPlanner::DebugLayer::DebugLayer(const QString &name, int importance):
	_planner(NULL),
	_name(name), _importance(importance), _minimumZoomFactor(0.0), _maximumZoomFactor(INFINITY)
{ }

AbstractPlanner::DebugLayer::~DebugLayer() {
	if(_planner) {
		_planner->removeDebugLayer(this);
		_planner = NULL;
	}
}
void AbstractPlanner::DebugLayer::draw(QPainter &p, const QRect &visibleArea, qreal zoomFactor) {
	if(_planner) _planner->drawDebugLayer(p, this, visibleArea, zoomFactor);
}

void AbstractPlanner::DebugLayer::setMinimumZoomFactor(qreal factor) {	
	_minimumZoomFactor = factor < 0.0 ? 0.0 : factor;
	if(_minimumZoomFactor > _maximumZoomFactor) _maximumZoomFactor = _minimumZoomFactor;
}
void AbstractPlanner::DebugLayer::setMaximumZoomFactor(qreal factor) {
	_maximumZoomFactor = factor < 0.0 ? 0.0 : factor;
	if(_minimumZoomFactor > _maximumZoomFactor) _minimumZoomFactor = _maximumZoomFactor;
}
void AbstractPlanner::DebugLayer::setZoomFactorRange(qreal minimum, qreal maximum) {
	_minimumZoomFactor = minimum < 0.0 ? 0.0 : minimum;
	_maximumZoomFactor = maximum < _minimumZoomFactor ? _minimumZoomFactor : maximum;	
}
