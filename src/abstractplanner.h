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

#ifndef ABSTRACTPLANNER_H
#define ABSTRACTPLANNER_H

#include <QObject>
#include "data.h"
#include <QList>
#include <QSize>
#include <QRect>
class QAction;
#include <QImage>
class QPainter;

class AbstractPlanner: public QObject {
	Q_OBJECT
public:	
	AbstractPlanner(QObject *parent = 0);
	virtual ~AbstractPlanner();

	
	class DebugLayer {
	public:
		DebugLayer(const QString &name, int importance = 1);
		virtual ~DebugLayer();
		virtual void draw(QPainter &p, const QRect &visibleArea, qreal zoomFactor);
		const QString &name() const { return _name; }
		int importance() const { return _importance; }
		qreal minimumZoomFactor() const { return _minimumZoomFactor; }
		qreal maximumZoomFactor() const { return _maximumZoomFactor; }
		void setMinimumZoomFactor(qreal factor);
		void setMaximumZoomFactor(qreal factor);
		void setZoomFactorRange(qreal minimum, qreal maximum);		
	private:		
		friend class AbstractPlanner;
		AbstractPlanner *_planner;
		
		QString _name;
		int _importance;
		qreal _minimumZoomFactor, _maximumZoomFactor;
	};
	
	typedef QList<DebugLayer *> DebugLayers;
	const DebugLayers &debugLayers() const { return _debugLayers; }
	
	enum ConfigElement {
		Element_DebugLayer,
		Element_Action,
		Element_Parameter
	};
	enum ConfigChange {
		Change_Add,
		Change_Modify,
		Change_Remove
	};
	
	void setStart(const Pose2D &start);
	void setStart(const QPointF &start);
	void setGoal(const Pose2D &goal);	
	void setGoal(const QPointF &goal);
	void setStartGoal(const Pose2D &start, const Pose2D &goal);
	void setStartGoal(const QPointF &start, const QPointF &goal);
	
	inline const Pose2D &start() const { return _start; }
	inline const QPointF &startPos() const { return _start.pos(); }
	inline const Pose2D &goal() const { return _goal; }
	inline const QPointF &goalPos() const { return _goal.pos(); }
	
	void setMap(const QImage &mapData);
	void updateMap(const QImage &mapData, const QRect &updateRegion);

	
	const Path &path() const { return _path; }
		
	int64_t calcTimeMs() const { return _calcTimeMs; }
	const QString &lastError() const { return _lastError; }
	
	QSize mapSize() const { return _mapSize; }
	int mapWidth() const { return _mapSize.width(); }
	int mapHeight() const { return _mapSize.height(); }

	enum InputUpdate {
		NoInputUpdates = 0,
		UpdatedStart = 1,
		UpdatedGoal = 2,
		UpdatedMap = 4,
		NewMap = 8
	};
		
	Q_DECLARE_FLAGS(InputUpdates, InputUpdate)
	
	QList<QAction *> actions() { return _actions; }
	
	virtual QString cellDetails(const QPoint &/*pos*/) { return QString(); }	
	
signals:
	void dataChanged();
	void configChanged(AbstractPlanner::ConfigElement element, AbstractPlanner::ConfigChange type, int index);
	void mapChanged(const QImage map);
	
protected:	
	/* call this to 
	 * - set a completely new map (updateRegion is empty)
	 * - update a portion of the current map
	 */
	virtual void initMap(const QImage &map, const QRect &updateRegion = QRect()) = 0;
	
	
	/* call this to calculate a new path after one or more input parameters have been changed	 
	 */
	
	virtual void calculatePath(InputUpdates updates) = 0;
	
	void setPath(const Path &path);
	void setError(const QString &str);
	void setError(const char *format, ...) __attribute__(( format(printf, 2, 3) ));


	void addDebugLayer(DebugLayer *layer, int after = -1);
	void addDebugLayer(DebugLayer *layer, DebugLayer *after);
	void removeDebugLayer(DebugLayer *layer);
	void addAction(QAction *action);


	virtual void drawDebugLayer(QPainter &, const DebugLayer *, const QRect &, qreal /*zoomFactor*/) { }

private:	
	Path _path;
	Pose2D _start, _goal;
	QSize _mapSize;
	QString _lastError;	
	
	void updatePath();
	int64_t _calcTimeMs;
	
	DebugLayers _debugLayers;
	bool inDestructor;
	
	InputUpdates accumulatedInputUpdates;
	void callPlanner();
	
	QList<QAction *> _actions;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(AbstractPlanner::InputUpdates)

#endif // ABSTRACTPLANNER_H
