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

#ifndef VISUALIZATIONWIDGET_H
#define VISUALIZATIONWIDGET_H

#include "zoomablewidget.h"
class QImage;
#include "abstractplanner.h"
#include "data.h"
#include <QAbstractListModel>
#include <QList>
#include <QCursor>

class VisualizationWidget: public ZoomableWidget {
	Q_OBJECT
public:
	VisualizationWidget(QWidget *parent = NULL);
	
	void setMap(const QImage &map);
	void setPlanner(AbstractPlanner *planner);
	
	const Pose2D &start() const { return _start; }
	const Pose2D &goal() const { return _goal; }
	void setStart(const Pose2D &start);
	void setGoal(const Pose2D &goal);
	
	QAbstractListModel *layerModel() { return _layerModel; }
	
	enum Tool {
		Tool_None,
		Tool_Pointer,
		Tool_Pen,
		Tool_Line,
		Tool_Rect
	};
	Tool tool() const { return _tool; }	
	int toolCost() const { return _toolCost; }
	
	class RLCPen {
	public:
		RLCPen() { _runs.push_back(Run(0, 0, 1, 255)); }
		virtual ~RLCPen() { }
		
		struct Run {
			Run(): y(0), xStart(0), xLength(0), cost(0) { }
			Run(short y, short xStart, unsigned short xLength = 1, unsigned char cost = 255): y(y), xStart(xStart), xLength(xLength), cost(cost) { }
			short y, xStart;
			unsigned short xLength;
			unsigned char cost;
			int xEnd() const { return xStart + xLength - 1; }
		};
		typedef QVector<struct Run> Runs;
		const Runs &runs() const { return _runs; }
		QImage toImage() const;
		QRect boundingRect() const;
		void setCost(unsigned char cost);
		
	protected:
		void setRuns(const Runs &runs);
	
	private:
		Runs _runs;
		mutable QRect _boundingRect;
		mutable QImage _image;
	};
	
	void setPen(const RLCPen &pen);
	const RLCPen &pen() const { return _pen; }
	
	
public slots:
	void setTool(VisualizationWidget::Tool tool);
	void setToolCost(int cost);

signals:
	void startPoseChanged();
	void goalPoseChanged();
	
protected:
	void paintContent(QPainter &painter);
	void paintOverlays(QPainter &, const QRect &area);
	
	void worldMousePressEvent(const QPointF &pos, Qt::MouseButtons buttons, Qt::MouseButton button);
	void worldMouseMoveEvent(const QPointF &pos, Qt::MouseButtons buttons);
	void worldMouseReleaseEvent(const QPointF &pos, Qt::MouseButtons buttons, Qt::MouseButton button);

private slots:
	void updatePlannerData();
	void handlePlannerConfigChanged(AbstractPlanner::ConfigElement element, AbstractPlanner::ConfigChange type, int index);
	void handleMapChangeFromPlanner(const QImage map);
	
private:
	QImage _map;
	AbstractPlanner *_planner;
	
	Pose2D _start, _goal;
	QPointF mouseDownPos;
	
	enum MouseObject{
		Mouse_Nothing,
		Mouse_Start,
		Mouse_Goal
	};
	MouseObject mouseObject;
	
	Qt::MouseButton mouseReleaseButton;
	
	friend class LayerModel;
	
	class LayerModel: public QAbstractListModel {
	public:
		LayerModel(VisualizationWidget *parent);		
	protected:
		int rowCount(const QModelIndex &parent = QModelIndex()) const;
		QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
		QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;
		Qt::ItemFlags flags(const QModelIndex &index) const;
		bool setData(const QModelIndex &index, const QVariant &value, int role);
	private:
		void emitDataChanged(const QModelIndex &from, const QModelIndex &to) { emit dataChanged(from, to); }
		void emitDataChanged(const QModelIndex &from) { emit dataChanged(from, from); }
		VisualizationWidget *vis;	
		friend class VisualizationWidget;	
	};
	
	LayerModel *_layerModel;

	enum LayerType {
		LayerType_Unknown,
		LayerType_Internal,
		LayerType_Planner			
	};
	enum InternalLayer {
		Layer_Map,
		Layer_Path,
		Layer_StartGoal
	};
	struct Layer {			
		LayerType type;
		union {
			InternalLayer internalLayer;
			AbstractPlanner::DebugLayer *plannerDebugLayer;
		};
		bool visible;			
		
		Layer(): type(LayerType_Unknown), visible(false) { }
		Layer(LayerType type): type(type), visible(true) { }
		Layer(InternalLayer layer): type(LayerType_Internal), visible(true) { internalLayer = layer; }
		Layer(AbstractPlanner::DebugLayer *layer): type(LayerType_Planner), visible(!!layer) { 
			plannerDebugLayer = layer;  
			if(layer && layer->importance() <= 0) visible = false;
		}
	};
	
	static const int roboRadius = 3;
	
	QList<Layer> layers;	
	
	Tool _activeTool;
	Tool _tool;
	unsigned char _toolCost;
	RLCPen _pen;
	
	QCursor penCursor;
	
	QRect toolBoundingRect;
	
	void addRect(const QRect &rc, unsigned char cost);
	void addLine(const QPoint &p1, const QPoint &p2, bool invert = false);
	void addPoint(int x, int y, bool invert = false);
};

#endif // VISUALIZATIONWIDGET_H
