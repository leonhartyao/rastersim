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

#include "simmainwindow.h"
#include <QAction>
#include <QActionGroup>
#include <QKeySequence>
#include <QShortcut>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QSettings>
#include <QApplication>
#include <QMenu>
#include <QMenuBar>
#include <QDockWidget>
#include <QToolBar>
#include <QMessageBox>
#include <QFileDialog>
#include <cstdio>
#include <QPixmap>
#include <QStatusBar>
#include <QSlider>
#include <QSpinBox>
#include "visualizationwidget.h"
#include <cmath>
#include "astarplanner.h"
#include "dstarplanner.h"
#include "fdstarplanner.h"
#include "dstarliteplanner.h"
#include <QDockWidget>
#include <QListView>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QToolButton>
#include "flowlayout.h"
#include "rlcpens.h"

#define INI_FILEPATH				"rastersim.ini"

#define REGKEY_GEOMETRY				"mainWindow/geometry"
#define REGKEY_STATE				"mainWindow/state"

#define REGKEY_MAPPATH				"map/path"
#define REGKEY_MAPFILE				"map/file"
#define REGKEY_VIZSTATE				"visualization/state"
#define REGKEY_EDIT_COST			"editor/cost"
#define REGKEY_EDIT_TOOL			"editor/tool"
#define REGKEY_EDIT_PENWIDTH		"editor/penwidth"
#define REGKEY_EDIT_PENSHAPE		"editor/penshape"

#define REGKEY_START_X				"robo/x"
#define REGKEY_START_Y				"robo/y"
#define REGKEY_START_ANGLE			"robo/angle"
#define REGKEY_GOAL_X				"goal/x"
#define REGKEY_GOAL_Y				"goal/y"
#define REGKEY_GOAL_ANGLE			"goal/angle"

#define REGKEY_PLANNER				"planner"

class FullInitFocussedDStarPlanner: public FocussedDStarPlanner {
public: 
	FullInitFocussedDStarPlanner(QObject *parent = 0): FocussedDStarPlanner(parent) { setFullInit(true); }
};

SimMainWindow::SimMainWindow(QWidget *parent):
	QMainWindow(parent), 
	planner(NULL)
{
	setWindowTitle(qApp->applicationName());
	//setWindowIcon(QIcon(tr(":/images/???.svg")));

	visualization = new VisualizationWidget;

	plannerFactories.push_back(new GenericPlannerFactory<AStarPlanner>("A-Star (A*)"));
	plannerFactories.push_back(new GenericPlannerFactory<DStarPlanner>("D-Star (D*)"));
	plannerFactories.push_back(new GenericPlannerFactory<FocussedDStarPlanner>("Focussed D* (FD*)"));
	plannerFactories.push_back(new GenericPlannerFactory<FullInitFocussedDStarPlanner>("FD* with full init."));
	plannerFactories.push_back(new GenericPlannerFactory<DStarLitePlanner>("D* Lite"));

	createActions();
	createToolbars();
	createDocks();
	createMenus();		
	
	setCentralWidget(visualization);

	zoomLabel = new QLabel(this);
	zoomLabel->setMinimumWidth(100);
	zoomLabel->setAlignment(Qt::AlignCenter);
	updateZoomFactor(visualization->zoomFactor());
	statusBar()->addWidget(zoomLabel);
	connect(visualization, SIGNAL(zoomFactorChanged(qreal)), this, SLOT(updateZoomFactor(qreal)));
	
	mouseCoordsLabel = new QLabel(this);
	mouseCoordsLabel->setMinimumWidth(150);
	statusBar()->addWidget(mouseCoordsLabel);	
	connect(visualization, SIGNAL(mousePosChanged(QPointF)), this, SLOT(updateMouseCoords(QPointF)));	
	
	startGoalLabel = new QLabel(this);
	startGoalLabel->setMinimumWidth(250);
	statusBar()->addWidget(startGoalLabel);
	connect(visualization, SIGNAL(startPoseChanged()), this, SLOT(updateStartGoal()));
	connect(visualization, SIGNAL(goalPoseChanged()), this, SLOT(updateStartGoal()));		
	
	// restore settings
	QSettings settings(INI_FILEPATH, QSettings::IniFormat);
	restoreGeometry(settings.value(REGKEY_GEOMETRY).toByteArray());
	restoreState(settings.value(REGKEY_STATE).toByteArray());
		
	lastMapDir = settings.value(REGKEY_MAPPATH).toString();
	lastMapFile = settings.value(REGKEY_MAPFILE).toString();
	
	visualization->restoreState(settings.value(REGKEY_VIZSTATE).toByteArray());	
	showOverlaysAction->setChecked(visualization->showOverlays());
	
	visualization->setToolCost(settings.value(REGKEY_EDIT_COST, 255).toInt());
	if(visualization->toolCost() > 128) maxCostAction->setChecked(true);
	else minCostAction->setChecked(true);
	visualization->setTool((VisualizationWidget::Tool)settings.value(REGKEY_EDIT_TOOL, VisualizationWidget::Tool_Pointer).toInt());
	switch(visualization->tool()) {
	case VisualizationWidget::Tool_Pen: penToolAction->setChecked(true); break;
	case VisualizationWidget::Tool_Line: lineToolAction->setChecked(true); break;
	case VisualizationWidget::Tool_Rect: rectToolAction->setChecked(true); break;
	default: pointerToolAction->setChecked(true);
	}
	
	penWidthSlider->setValue(settings.value(REGKEY_EDIT_PENWIDTH, 5).toInt());	
	penStyleCombo->setCurrentIndex(settings.value(REGKEY_EDIT_PENSHAPE, 0).toInt());
	updatePenFromControls();
	
	mapFreeColor = qRgb(255, 255, 255);
	mapFreeColorTolerance = 10;
		
	if(!lastMapFile.isEmpty()) {
		// try to load map file
		loadMap(QDir(lastMapDir).filePath(lastMapFile));
	}
	
	Pose2D start(settings.value(REGKEY_START_X, NAN).toDouble(),
				 settings.value(REGKEY_START_Y, NAN).toDouble(),
				 settings.value(REGKEY_START_ANGLE, NAN).toDouble());

	if(start.isValid()) visualization->setStart(start);
	Pose2D goal(settings.value(REGKEY_GOAL_X, NAN).toDouble(),
				settings.value(REGKEY_GOAL_Y, NAN).toDouble(),
				settings.value(REGKEY_GOAL_ANGLE, NAN).toDouble());
	if(goal.isValid()) visualization->setGoal(goal);
	
	updateStartGoal();
		
	plannerCombo->setCurrentIndex(settings.value(REGKEY_PLANNER, 0).toInt());
}

SimMainWindow::~SimMainWindow() {
	qDeleteAll(plannerFactories);
}

void SimMainWindow::closeEvent(QCloseEvent *) {
	QSettings settings(INI_FILEPATH, QSettings::IniFormat);
	
	settings.setValue(REGKEY_GEOMETRY, saveGeometry());
	settings.setValue(REGKEY_STATE, saveState());	
	
	settings.setValue(REGKEY_MAPPATH, lastMapDir);
	settings.setValue(REGKEY_MAPFILE, lastMapFile);
	settings.setValue(REGKEY_VIZSTATE, visualization->saveState());
	settings.setValue(REGKEY_EDIT_COST, visualization->toolCost());
	settings.setValue(REGKEY_EDIT_TOOL, visualization->tool());
	settings.setValue(REGKEY_EDIT_PENWIDTH, penWidthSlider->value());
	settings.setValue(REGKEY_EDIT_PENSHAPE, penStyleCombo->currentIndex());
	
	settings.setValue(REGKEY_START_X, visualization->start().x());
	settings.setValue(REGKEY_START_Y, visualization->start().y());
	settings.setValue(REGKEY_START_ANGLE, visualization->start().angle());	
	settings.setValue(REGKEY_GOAL_X, visualization->goal().x());
	settings.setValue(REGKEY_GOAL_Y, visualization->goal().y());
	settings.setValue(REGKEY_GOAL_ANGLE, visualization->goal().angle());	
	settings.setValue(REGKEY_PLANNER, plannerCombo->currentIndex());
}

void SimMainWindow::createActions() {
	openMapAction = new QAction(tr("Open Map..."), this);
	openMapAction->setShortcut(Qt::CTRL + Qt::Key_O);
	connect(openMapAction, SIGNAL(triggered(bool)), this, SLOT(openMap()));	
	
	minCostAction = new QAction(QIcon(tr(":images/color_white.svg")), trUtf8("Draw Free Space"), this);
	minCostAction->setCheckable(true);
	maxCostAction = new QAction(QIcon(tr(":images/color_black.svg")), trUtf8("Draw Obstacles"), this);
	maxCostAction->setCheckable(true);	
	costActions = new QActionGroup(this);
	costActions->setExclusive(true);
	costActions->addAction(minCostAction);
	costActions->addAction(maxCostAction);
	connect(costActions, SIGNAL(triggered(QAction *)), this, SLOT(changeCost(QAction *)));
	
	pointerToolAction = new QAction(QIcon(tr(":images/tool_pointer.svg")), tr("Move Start/Goal"), this);
	pointerToolAction->setCheckable(true);
	penToolAction = new QAction(QIcon(tr(":images/tool_pen.svg")), tr("Draw freehand"), this);
	penToolAction->setCheckable(true);
	lineToolAction = new QAction(QIcon(tr(":images/tool_line.svg")), tr("Draw straight line"), this);
	lineToolAction->setCheckable(true);
	rectToolAction = new QAction(QIcon(tr(":images/tool_rect.svg")), tr("Draw rectangular shape"), this);
	rectToolAction->setCheckable(true);
	toolActions = new QActionGroup(this);
	toolActions->setExclusive(true);
	toolActions->addAction(pointerToolAction);
	toolActions->addAction(penToolAction);
	toolActions->addAction(lineToolAction);
	toolActions->addAction(rectToolAction);	
	connect(toolActions, SIGNAL(triggered(QAction *)), this, SLOT(changeTool(QAction *)));
	
	QShortcut *rotLeftShortcut = new QShortcut(Qt::CTRL | Qt::Key_Left, this);
	connect(rotLeftShortcut, SIGNAL(activated()), this, SLOT(rotateLeft()));
	QShortcut *rotRightShortcut = new QShortcut(Qt::CTRL | Qt::Key_Right, this);
	connect(rotRightShortcut, SIGNAL(activated()), this, SLOT(rotateRight()));
		
	showOverlaysAction = new QAction(trUtf8("Overlays"), this);
	showOverlaysAction->setCheckable(true);
	connect(showOverlaysAction, SIGNAL(toggled(bool)), visualization, SLOT(setShowOverlays(bool)));
}

void SimMainWindow::createToolbars() {	
	mapToolBar = new QToolBar(tr("Map tools"));
	mapToolBar->setObjectName("maptools");
	
	mapToolBar->addAction(pointerToolAction);
	mapToolBar->addAction(penToolAction);
	mapToolBar->addAction(lineToolAction);
	mapToolBar->addAction(rectToolAction);
	mapToolBar->addSeparator();

	mapToolBar->addAction(minCostAction);
	mapToolBar->addAction(maxCostAction);
	mapToolBar->addSeparator();
	
	QLabel *penStyleCaptionLabel = new QLabel(tr("Pen &Style: "));
	mapToolBar->addWidget(penStyleCaptionLabel);
	penStyleCombo = new QComboBox;
	penStyleCombo->setIconSize(QSize(24, 24));
	penStyleCombo->addItem(QIcon(QPixmap::fromImage(RLCPens::SquarePen(20).toImage())), "", PenStyle_Square);	
	penStyleCombo->addItem(QIcon(QPixmap::fromImage(RLCPens::CirclePen(20).toImage())), "", PenStyle_Circle);
	penStyleCombo->addItem(QIcon(QPixmap::fromImage(RLCPens::DiamondPen(20).toImage())), "", PenStyle_Diamond);
	penStyleCombo->addItem(QIcon(QPixmap::fromImage(RLCPens::BarPen(20, Qt::Vertical).toImage())), "", PenStyle_VBar);
	penStyleCombo->addItem(QIcon(QPixmap::fromImage(RLCPens::BarPen(20, Qt::Horizontal).toImage())), "", PenStyle_HBar);
	connect(penStyleCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updatePenFromControls()));
		
	mapToolBar->addWidget(penStyleCombo);
	penStyleCaptionLabel->setBuddy(penStyleCombo);
	QLabel *penWidthCaptionLabel = new QLabel(tr(" &Width: "));
	mapToolBar->addWidget(penWidthCaptionLabel);
	penWidthSlider = new QSlider(Qt::Horizontal);
	penWidthSlider->setRange(1, 200);
	penWidthSlider->setSingleStep(1);
	penWidthSlider->setPageStep(15);
	penWidthSlider->setMaximumWidth(200);
	mapToolBar->addWidget(penWidthSlider);
	penWidthCaptionLabel->setBuddy(penWidthSlider);
	penWidthSpinBox = new QSpinBox;
	penWidthSpinBox->setRange(penWidthSlider->minimum(), penWidthSlider->maximum());
	penWidthSpinBox->setSingleStep(penWidthSlider->singleStep());
	mapToolBar->addWidget(penWidthSpinBox);
	connect(penWidthSlider, SIGNAL(valueChanged(int)), penWidthSpinBox, SLOT(setValue(int)));
	connect(penWidthSpinBox, SIGNAL(valueChanged(int)), penWidthSlider, SLOT(setValue(int)));		
	connect(penWidthSlider, SIGNAL(valueChanged(int)), this, SLOT(updatePenFromControls()));
	
	addToolBar(Qt::TopToolBarArea, mapToolBar);
	
	
	viewToolBar = new QToolBar(tr("View"));
	viewToolBar->setObjectName("viewtools");
		
	viewToolBar->addAction(visualization->zoomInAction());
	viewToolBar->addAction(visualization->zoomOutAction());
	viewToolBar->addAction(visualization->zoomResetAction());
	viewToolBar->addAction(showOverlaysAction);
	viewToolBar->addSeparator();

	viewToolBar->addAction(visualization->rotate90CCWAction());
	viewToolBar->addAction(visualization->rotate0Action());
	viewToolBar->addAction(visualization->rotate90CWAction());
	viewToolBar->addAction(visualization->rotate180Action());
	viewToolBar->addSeparator();

	viewToolBar->addAction(visualization->mirrorHAction());
	viewToolBar->addAction(visualization->mirrorVAction());
	
	addToolBar(Qt::TopToolBarArea, viewToolBar);
}

void SimMainWindow::createDocks() {
	QVBoxLayout *optMainLayout = new QVBoxLayout;
	
	plannerCombo = new QComboBox;
	for(int i = 0; i < plannerFactories.size(); i++) plannerCombo->addItem(plannerFactories[i]->name());
	plannerCombo->setCurrentIndex(-1);
	connect(plannerCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(setPlanner(int)));
	optMainLayout->addWidget(plannerCombo, 0);
	
	layerView = new QListView;
	layerView->setModel(visualization->layerModel());
	optMainLayout->addWidget(layerView, 1);	
	
	plannerActionsLayout = new FlowLayout;
	optMainLayout->addLayout(plannerActionsLayout);
	
	QHBoxLayout *optCalcTimeLayout = new QHBoxLayout;
	QLabel *calcTimeCaptionLabel = new QLabel(tr("calc. time:"));
	calcTimeCaptionLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
	optCalcTimeLayout->addWidget(calcTimeCaptionLabel, 0);
	calcTimeLabel = new QLabel("---");
	calcTimeLabel->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
	QFont font = calcTimeLabel->font();
	font.setPointSize(14);
	font.setBold(true);
	calcTimeLabel->setFont(font);
	optCalcTimeLayout->addWidget(calcTimeLabel, 1);
	optMainLayout->addLayout(optCalcTimeLayout, 0);
	
	cellDetailLabel = new QLabel;
	cellDetailLabel->setMinimumHeight(100);
	cellDetailLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
	optMainLayout->addWidget(cellDetailLabel);	
	
	QWidget *optionsWidget = new QWidget;
	optionsWidget->setLayout(optMainLayout);
	optionsDock = new QDockWidget(tr("Planner && Visualization Options"));	
	optionsDock->setObjectName("options_dock");
	optionsDock->setWidget(optionsWidget);	
	addDockWidget(Qt::RightDockWidgetArea, optionsDock);
}
void SimMainWindow::createMenus() {
	QMenu *fileMenu = new QMenu(tr("File"), this);
	fileMenu->addAction(openMapAction);
	fileMenu->addSeparator();
	fileMenu->addAction(tr("Quit"), this, SLOT(close()), Qt::ALT + Qt::Key_F4);
	menuBar()->addMenu(fileMenu); 
	
	QMenu *viewMenu = new QMenu(tr("View"), this);
	viewMenu->addAction(mapToolBar->toggleViewAction());
	viewMenu->addAction(viewToolBar->toggleViewAction());
	menuBar()->addMenu(viewMenu);
	
	QMenu *infoMenu = new QMenu(tr("&?"));
	infoMenu->addAction(tr("About..."), this, SLOT(showAbout()), Qt::Key_F1);
	infoMenu->addAction(tr("About Qt..."), qApp, SLOT(aboutQt()), Qt::CTRL + Qt::Key_F1);
	menuBar()->addMenu(infoMenu);
}

void SimMainWindow::updateMouseCoords(QPointF pt) {
	QPoint pt_i = pt.toPoint();
	mouseCoordsLabel->setText(QString("X = %1, Y = %2").arg(pt_i.x()).arg(pt_i.y()));
	
	if(planner) cellDetailLabel->setText(planner->cellDetails(pt_i));
}
void SimMainWindow::updateZoomFactor(qreal factor) {
	factor *= 100;
	zoomLabel->setText(QString(trUtf8("%1 %")).arg(factor, 0, 'f', qMax(0, 2 - (int)log10(factor))));
}

void SimMainWindow::updateStartGoal() {
	Pose2D start = visualization->start();
	Pose2D goal = visualization->goal();
	
	QString startString = (start.isValid() ? 
		QString("(%1; %2)").arg(start.x(), 0, 'f', qMax(0, 2 - (int)log10(start.x()))).arg(start.y(), 0, 'f', qMax(0, 2 - (int)log10(start.y()))) 
		: "?");
	QString goalString = (goal.isValid() ? 
		QString("(%1; %2)").arg(goal.x(), 0, 'f', qMax(0, 2 - (int)log10(goal.x()))).arg(goal.y(), 0, 'f', qMax(0, 2 - (int)log10(goal.y())))
		: "?");
	startGoalLabel->setText(startString + " -> " + goalString);	
}

void SimMainWindow::updatePlannerData() {
	if(planner) {
		int64_t calcTime = planner->calcTimeMs();
		if(calcTime < 0) calcTimeLabel->setText("---");
		else if(calcTime < 1000) calcTimeLabel->setText(QString("%1 ms").arg(calcTime));
		else {
			qreal calcTimef = (qreal)calcTime / 1000.0;
			calcTimeLabel->setText(QString("%1 s").arg(calcTimef, 0, 'f', qMax(0, 2  - (int)log10(calcTimef))));
		}
	}
}
void SimMainWindow::rotateLeft() {
	switch(visualization->rotation()) {
	case ZoomableWidget::Rotate_90CCW: visualization->setRotation(ZoomableWidget::Rotate_180); break;
	case ZoomableWidget::Rotate_None: visualization->setRotation(ZoomableWidget::Rotate_90CCW); break;
	case ZoomableWidget::Rotate_90CW: visualization->setRotation(ZoomableWidget::Rotate_None); break;
	case ZoomableWidget::Rotate_180: visualization->setRotation(ZoomableWidget::Rotate_90CW); break;
	}
}
void SimMainWindow::rotateRight() {
	switch(visualization->rotation()) {
	case ZoomableWidget::Rotate_90CCW: visualization->setRotation(ZoomableWidget::Rotate_None); break;
	case ZoomableWidget::Rotate_None: visualization->setRotation(ZoomableWidget::Rotate_90CW); break;
	case ZoomableWidget::Rotate_90CW: visualization->setRotation(ZoomableWidget::Rotate_180); break;
	case ZoomableWidget::Rotate_180: visualization->setRotation(ZoomableWidget::Rotate_90CCW); break;
	}
}

void SimMainWindow::openMap() {
	QString fileName = QFileDialog::getOpenFileName(this, tr("select map..."), QDir(lastMapDir).filePath(lastMapFile), tr("image files (*.bmp *.jpg *.png)\nall files (*.*)")); 	
	if(!fileName.isEmpty()) loadMap(fileName);
}

bool SimMainWindow::loadMap(const QString &fileName) {
	bool success = false;
	QImage img(fileName);
	if(!img.isNull()) {
		QImage map(img.size(), QImage::Format_Indexed8);
		QVector<QRgb> colorTable(256);
		for(unsigned i = 0; i < 256; i++) colorTable[255 - i] = qRgb(i, i, i);
		map.setColorTable(colorTable);
		
		img = img.convertToFormat(QImage::Format_RGB32);		
		for(int y = 0; y < img.height(); y++) {
			unsigned char *pDest = (unsigned char*)map.scanLine(y);
			const QRgb *pSrc = (const QRgb *)img.scanLine(y);
			for(int x = 0; x < img.width(); x++) {
				QRgb clr = *pSrc++;
				int rDelta = qAbs(qRed(mapFreeColor) - qRed(clr));
				int gDelta = qAbs(qGreen(mapFreeColor) - qGreen(clr));
				int bDelta = qAbs(qBlue(mapFreeColor) - qBlue(clr));
				if(gDelta > rDelta) rDelta = gDelta;
				if(bDelta > rDelta) rDelta = bDelta;
				
				if(rDelta > mapFreeColorTolerance) *pDest = 255;
				else *pDest = 0;				
			
				pDest++;
			} 			
		}
		
		visualization->setMap(map);		
			
		QFileInfo fi(fileName);
		lastMapDir = fi.path();
		lastMapFile = fi.fileName();

		success = true;
	} else QMessageBox::warning(this, qApp->applicationName(), QString(tr("Could not load map file \"%1\"")).arg(fileName));
	
	return success;
}

void SimMainWindow::setPlanner(int index) {
	if(index < 0 || index >= plannerFactories.size()) return;
	
	AbstractPlanner *oldPlanner = planner;
	if(oldPlanner) oldPlanner->disconnect();
	
	planner = plannerFactories[index]->create(this);	
	connect(planner, SIGNAL(dataChanged()), this, SLOT(updatePlannerData()));
	visualization->setPlanner(planner);
	
	for(int i = plannerActionsLayout->count() - 1; i >= 0; i--) {
		plannerActionsLayout->itemAt(i)->widget()->deleteLater();
	}
	foreach(QAction *a, planner->actions()) {
		QToolButton *btn = new QToolButton;
		btn->setDefaultAction(a);
		btn->setAutoRaise(true);
		plannerActionsLayout->addWidget(btn);
	}
	
	
	delete oldPlanner;	
}

void SimMainWindow::changeTool(QAction *toolAction) {
	visualization->setTool(toolAction == penToolAction ? VisualizationWidget::Tool_Pen :
						   toolAction == lineToolAction ? VisualizationWidget::Tool_Line :
						   toolAction == rectToolAction ? VisualizationWidget::Tool_Rect :
						   VisualizationWidget::Tool_Pointer);
}
void SimMainWindow::changeCost(QAction *costAction) {
	visualization->setToolCost(costAction == minCostAction ? 0 : 255);
	VisualizationWidget::RLCPen pen = visualization->pen();
	pen.setCost(visualization->toolCost());
	visualization->setPen(pen);
}

	void SimMainWindow::updatePenFromControls() {
	int penSize = penWidthSlider->value();
	VisualizationWidget::RLCPen pen;
	switch((PenStyle)penStyleCombo->itemData(penStyleCombo->currentIndex()).toInt()) {
	case PenStyle_Square: pen = RLCPens::SquarePen(penSize); break;
	case PenStyle_Diamond: pen = RLCPens::DiamondPen(penSize); break;
	case PenStyle_VBar: pen = RLCPens::BarPen(penSize, Qt::Vertical); break;
	case PenStyle_HBar: pen = RLCPens::BarPen(penSize, Qt::Horizontal); break;
	case PenStyle_Circle: 
	default:
		pen = RLCPens::CirclePen(penSize);
	}
	pen.setCost(visualization->toolCost());
	visualization->setPen(pen);
}

void SimMainWindow::showAbout(){
	QMessageBox::about(this, qApp->applicationName(), QString(trUtf8(
		"<CENTER><H3>TU Dresden</H3><I>Institut für Automatisierungstechnik</I><H1>"
		"RasterSim - %1</H1></CENTER>"
		"<P>Test utility for various raster path planners</P>"
		"<P>Author: Martin Seemann</P>"
		"<P>This program is released under terms of the GNU General Pulic License (GPL), 2nd version: "
		"<A href = \"http://www.gnu.org/licenses/gpl-2.0.html\">www.gnu.org/licenses/gpl-2.0.html</A></P>"
	)).arg(qApp->applicationVersion()));	
}

