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

#ifndef ZOOMABLEWIDGET_H
#define ZOOMABLEWIDGET_H

#include <QAbstractScrollArea>
#include <QByteArray>
#include <QSizeF>
#include <QPointF>
#include <QFlags>
class QAction;
class QActionGroup;

class ZoomableWidget: public QAbstractScrollArea {
	Q_OBJECT
public:
	ZoomableWidget(QWidget *parent = 0);

	QByteArray saveState();
	bool restoreState(const QByteArray &);	

	void setBackground(QColor color);
	qreal zoomFactor() const { return _zoomFactor; }
	QAction *zoomInAction();
	QAction *zoomOutAction();
	QAction *zoomResetAction();

	enum Rotation {
		Rotate_None,
		Rotate_90CCW,
		Rotate_90CW,
		Rotate_180
	};
	Rotation rotation() const { return _rotation; }
	QAction *rotate0Action();
	QAction *rotate90CWAction();
	QAction *rotate90CCWAction();
	QAction *rotate180Action();

	
	enum MirrorMode {
		Mirror_None = 0x00,
		Mirror_Horizontal = 0x01,
		Mirror_Vertical = 0x02,		
	};
	Q_DECLARE_FLAGS(MirrorModes, MirrorMode)
	MirrorModes mirrorMode() const { return _mirrorMode; }
	QAction *mirrorHAction();
	QAction *mirrorVAction();

	bool showOverlays() const { return _showOverlays; }
	QAction *overlaysAction();
	
public slots:	
	virtual void clear();
	
	void setRotation(Rotation rotation);
	void setMirrorMode(MirrorModes mode);
	void setHMirror(bool enable);
	void setVMirror(bool enable);	
	void setShowOverlays(bool enable);
	
    // zoom & pan
    void setZoomFactor(qreal newZoomFactor);
	void resetZoom();
	void zoomIn();
	void zoomOut();
	void scrollBy(int x, int y); // x, y in screen coordinates
    
    void setFullscreen(bool enabled);
    
signals:
	void zoomFactorChanged(qreal zoomFactor);
	void mousePosChanged(QPointF pt);
	
protected:
	virtual void paintContent(QPainter &) = 0;
	virtual void paintOverlays(QPainter &, const QRect &) { }
	virtual void paintEmptyContent(QPainter &, const QRect &area);
	void setWorld(const QSizeF &size, const QPointF &offset = QPointF(), qreal scaleFactor = 1.0);

	void updateContent();
	QPoint worldToWidget(QPointF ptWorld);
	QPointF widgetToWorld(QPoint ptWidget);
	
	virtual void worldMousePressEvent(const QPointF &, Qt::MouseButtons, Qt::MouseButton) { }
	virtual void worldMouseMoveEvent(const QPointF &, Qt::MouseButtons) { }
	virtual void worldMouseReleaseEvent(const QPointF &, Qt::MouseButtons, Qt::MouseButton) { }
	

	void paintEvent(QPaintEvent *);
    void resizeEvent(QResizeEvent *);
    void wheelEvent(QWheelEvent *);
    void mouseMoveEvent(QMouseEvent *);
	void mousePressEvent(QMouseEvent *);  
	void mouseReleaseEvent(QMouseEvent *);

private slots:
	void changeRotationFromAction(QAction *action);
	
private:
	qreal _zoomFactor;
	qreal _minZoom, _maxZoom;
	Rotation _rotation;
	MirrorModes _mirrorMode;
	
	enum TransformMode {
		Transform_None, 		// no rotation, no mirror  - or - 180° + H/V mirror			(1)
		Transform_HMirror,		// no rotation, H mirror   - or - 180° + V mirror			(2)
		Transform_VMirror,		// no rotation, V mirror   - or - 180° + H mirror			(3)
		Transform_90, 			// 90° CCW, no mirror      - or - 90° CW + H/V mirror		(5)
		Transform_90_HMirror, 	// 90° CCW, H mirror       - or - 90° CW + V mirror			(6)
		Transform_90_VMirror,	// 90° CCW, V mirror       - or - 90° CW + H mirror			(7)
		Transform_180, 			// 180°, no mirror         - or - no rotation + H/V mirror	(4)
		Transform_270, 			// 90° CW, no mirror       - or - 90° CCW + H/V mirror		(8)
	};
	TransformMode _transformMode;
	TransformMode calcTransformMode();
	
	int backupScrollPosH, backupScrollPosV; // storage for scroll positions while we have no content 
		
	void setZoomFactor(qreal newZoomFactor, QPoint center);

	bool _showOverlays;
	
	// returns content's top left position in widget coordinates 
	// (same as contentToWidget(0, 0), used internally by contentToWidget)
	QPoint contentTopLeft() const; 
	QPointF widgetToContent(const QPoint &) const; 
	QPoint contentToWidget(const QPointF &) const;
	QPointF contentToWorld(const QPointF &pt) const;
	QPointF worldToContent(const QPointF &pt) const;
	
	QPoint panStartPos;
	
	void updateScrollBars();		
	
	struct {
		QSizeF size;
		QPointF offset;
		qreal scaleFactor;
		qreal invScaleFactor;
		bool isEmpty() const { return size.isEmpty(); }
	} world;
	QSizeF contentSize;
	QSizeF calcContentSize() const;
	
	QActionGroup *_zoomActionGroup;
	QActionGroup *zoomActionGroup();
	QAction *_zoomInAction;
	QAction *_zoomOutAction;
	QAction *_zoomResetAction;
	QActionGroup *_rotateActionGroup;
	QActionGroup *rotateActionGroup();
	QAction *_rotate0Action;
	QAction *_rotate90CWAction;
	QAction *_rotate90CCWAction;
	QAction *_rotate180Action;	
	QActionGroup *_mirrorActionGroup;
	QActionGroup *mirrorActionGroup();
	QAction *_mirrorHAction;
	QAction *_mirrorVAction;		
	
	QAction *_overlaysAction;
	
};

Q_DECLARE_OPERATORS_FOR_FLAGS(ZoomableWidget::MirrorModes)

#endif // ZOOMABLEWIDGET_H
