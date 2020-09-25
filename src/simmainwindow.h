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

#ifndef SIMMAINWINDOW_H
#define SIMMAINWINDOW_H

#include <QMainWindow>

class QAction;
class QActionGroup;
class QLabel;
class QComboBox;
class QSlider;
class QSpinBox;
class QToolBar;
class VisualizationWidget;
class AbstractPlanner;
class QDockWidget;
class QListView;
class FlowLayout;

class SimMainWindow: public QMainWindow{
	Q_OBJECT
public:
	SimMainWindow(QWidget *parent = 0);
	~SimMainWindow();
	
protected:
	void closeEvent(QCloseEvent *);

private slots:
	void openMap();	
	void showAbout();

	void rotateLeft();
	void rotateRight();

	void setPlanner(int index);
	void updateMouseCoords(QPointF pt);
	void updateZoomFactor(qreal factor);
	void updateStartGoal();
	void updatePlannerData();
	
	void changeTool(QAction *);
	void changeCost(QAction *);
	void updatePenFromControls();
	
private:
	void createActions();
	void createToolbars();
	void createDocks();
	void createMenus();
	
	QToolBar *mapToolBar;
	QToolBar *viewToolBar;
	
	QAction *openMapAction;
	bool loadMap(const QString &fileName);
		
	QLabel *cursorPosLabel;
	QLabel *zoomLabel;
	
	QString lastMapDir;
	QString lastMapFile;
	
	VisualizationWidget *visualization;
	QLabel *mouseCoordsLabel;
	
	QLabel *startGoalLabel;
	
	QRgb mapFreeColor;
	int mapFreeColorTolerance;

	QAction *showOverlaysAction;
	
	AbstractPlanner *planner;
	
	QDockWidget *optionsDock;
	QComboBox *plannerCombo;
	QListView *layerView;
	QLabel *calcTimeLabel;
	QLabel *cellDetailLabel;
	FlowLayout *plannerActionsLayout;
	
	class PlannerFactoryBase {
	public:
		PlannerFactoryBase(const QString &name = QString()): _name(name) { }
		virtual ~PlannerFactoryBase() { }
		const QString &name() const { return _name; }
		virtual AbstractPlanner *create(QObject * = 0) = 0;		
	private:
		QString _name;
	};
	template <typename T>
	class GenericPlannerFactory: public PlannerFactoryBase {
	public:
		GenericPlannerFactory(const QString &name = QString()): PlannerFactoryBase(name) { }
		AbstractPlanner *create(QObject *parent = 0) { return new T(parent); }
	};
	
	QList<PlannerFactoryBase *> plannerFactories;	
	
	QActionGroup *costActions;
	QAction *minCostAction;
	QAction *maxCostAction;
	
	QActionGroup *toolActions;
	QAction *pointerToolAction;
	QAction *penToolAction;
	QAction *lineToolAction;
	QAction *rectToolAction;
	
	QComboBox *penStyleCombo;
	QSlider *penWidthSlider;
	QSpinBox *penWidthSpinBox;	
	
	enum PenStyle {
		PenStyle_Square,
		PenStyle_Circle,
		PenStyle_Diamond,
		PenStyle_VBar,
		PenStyle_HBar
	};
	
	
};

#endif // SIMMAINWINDOW_H
