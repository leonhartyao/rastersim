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

#include "zoomablewidget.h"
#include <QPalette>
#include <QPainter>
#include <QMouseEvent>
#include <QScrollBar>
#include <QAction>
#include <QActionGroup>

ZoomableWidget::ZoomableWidget(QWidget *parent):
	QAbstractScrollArea(parent),
	_zoomFactor(1.0),
	_minZoom(0.0625), _maxZoom(32.0),
	_rotation(Rotate_None),
	_mirrorMode(Mirror_None),
	_transformMode(Transform_None),	
	backupScrollPosH(0), backupScrollPosV(0),
	_showOverlays(true),
	_zoomActionGroup(NULL), _zoomInAction(NULL), _zoomOutAction(NULL), _zoomResetAction(NULL),
	_rotateActionGroup(NULL), _rotate0Action(NULL), _rotate90CWAction(NULL), _rotate90CCWAction(NULL), _rotate180Action(NULL),
	_mirrorActionGroup(NULL), _mirrorHAction(NULL), _mirrorVAction(NULL)
{
	setMouseTracking(true);
}

void ZoomableWidget::setBackground(QColor color) {
	QPalette pal = viewport()->palette();
	pal.setColor(QPalette::Background, color);
	viewport()->setPalette(pal);
	viewport()->setBackgroundRole(QPalette::Background);
}

static const quint32 storageFormatMagic = 0x56320C76;

QByteArray ZoomableWidget::saveState() {
    const quint32 version = 0;    
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);
    
    stream << storageFormatMagic << version << (float)_zoomFactor << (int)_rotation << (int)_mirrorMode << _showOverlays;
    if(contentSize.isEmpty()) stream << backupScrollPosH << backupScrollPosV;
    else stream << horizontalScrollBar()->value() << verticalScrollBar()->value();
	
	return data;
}

bool ZoomableWidget::restoreState(const QByteArray &ba) {
	quint32 magic, version;
    QByteArray nonConstArray = ba;
	QDataStream stream(&nonConstArray, QIODevice::ReadOnly);
    stream >> magic >> version;
    if(stream.status() != QDataStream::Ok) return false;
    if(magic != storageFormatMagic || version != 0) return false;
    
    float fZoomFactor;
    int rotation;
    int mirrorMode;
    bool showOverlays;
	int scrollPosH, scrollPosV;
	stream >> fZoomFactor >> rotation >> mirrorMode >> showOverlays >> scrollPosH >> scrollPosV;
	if(stream.status() != QDataStream::Ok) return false;
	
	setZoomFactor((qreal)fZoomFactor);
	setRotation((Rotation)rotation);
	setMirrorMode((MirrorModes)mirrorMode);
	setShowOverlays(showOverlays);
	backupScrollPosH = scrollPosH;
	backupScrollPosV = scrollPosV;
	horizontalScrollBar()->setValue(scrollPosH);
	verticalScrollBar()->setValue(scrollPosV);

	return true;		
}
	
void ZoomableWidget::clear() {
	setWorld(QSize());
}
	
void ZoomableWidget::setFullscreen(bool enabled) {
	// TBD
	Q_UNUSED(enabled)	
}

void ZoomableWidget::setRotation(Rotation rotation) {
	if(rotation == _rotation) return;
	
	_rotation = rotation;
	
	contentSize = calcContentSize();
	if(_rotate0Action) _rotate0Action->setChecked(_rotation == Rotate_None);
	if(_rotate90CWAction) _rotate90CWAction->setChecked(_rotation == Rotate_90CW);
	if(_rotate180Action) _rotate180Action->setChecked(_rotation == Rotate_180);
	if(_rotate90CCWAction) _rotate90CCWAction->setChecked(_rotation == Rotate_90CCW);
	
	_transformMode = calcTransformMode();
	if(!world.isEmpty()) {
		updateScrollBars();
		viewport()->update();
	}
}

void ZoomableWidget::setHMirror(bool enable) {
	if(enable) setMirrorMode(_mirrorMode | Mirror_Horizontal);
	else setMirrorMode(_mirrorMode & ~Mirror_Horizontal);
}
void ZoomableWidget::setVMirror(bool enable) {
	if(enable) setMirrorMode(_mirrorMode | Mirror_Vertical);
	else setMirrorMode(_mirrorMode & ~Mirror_Vertical);	
}
void ZoomableWidget::setMirrorMode(MirrorModes mode) {
	if(mode == _mirrorMode) return;
	
	_mirrorMode = mode;	
	if(_mirrorHAction) _mirrorHAction->setChecked(_mirrorMode & Mirror_Horizontal);
	if(_mirrorVAction) _mirrorVAction->setChecked(_mirrorMode & Mirror_Vertical);
	_transformMode = calcTransformMode();
	if(!world.isEmpty()) viewport()->update();
}

void ZoomableWidget::setShowOverlays(bool enable) {
	if(_showOverlays == enable) return;
	
	_showOverlays = enable;
	if(!world.isEmpty()) viewport()->update();
}

void ZoomableWidget::setZoomFactor(qreal newZoomFactor) {
	setZoomFactor(newZoomFactor, viewport()->rect().center());	
}
void ZoomableWidget::resetZoom() {
	setZoomFactor(1.0);
}
void ZoomableWidget::zoomIn() {
	setZoomFactor(2.0 * _zoomFactor);
}
void ZoomableWidget::zoomOut() {
	setZoomFactor(0.5 * _zoomFactor);
}
void ZoomableWidget::scrollBy(int x, int y) {
	horizontalScrollBar()->setValue(horizontalScrollBar()->value() + x);
	verticalScrollBar()->setValue(verticalScrollBar()->value() + y);	
}
    
void ZoomableWidget::setWorld(const QSizeF &size, const QPointF &offset, qreal scaleFactor) {
	bool wasEmpty = contentSize.isEmpty();
	world.size = size;
	world.offset = offset;
	world.scaleFactor = scaleFactor;
	world.invScaleFactor = 1.0 / scaleFactor;
	contentSize = calcContentSize();
	
	bool nowEmpty = contentSize.isEmpty();
	
	if(wasEmpty && nowEmpty) return;
	bool restoreScrollPos = wasEmpty && !nowEmpty;
	if(nowEmpty){		
		backupScrollPosH = horizontalScrollBar()->value();
		backupScrollPosV = verticalScrollBar()->value();
	}		
	
	updateScrollBars();
	if(restoreScrollPos){
		horizontalScrollBar()->setValue(backupScrollPosH);
		verticalScrollBar()->setValue(backupScrollPosV);
	}
	if(_zoomActionGroup) _zoomActionGroup->setEnabled(!nowEmpty);
	if(_rotateActionGroup) _rotateActionGroup->setEnabled(!nowEmpty);
	if(_mirrorActionGroup) _mirrorActionGroup->setEnabled(!nowEmpty);
	viewport()->update();	
}

QSizeF ZoomableWidget::calcContentSize() const {
	QSizeF sz = world.size * world.scaleFactor;
	if(_rotation == Rotate_90CW || _rotation == Rotate_90CCW) sz.transpose();
	return sz;	
}

void ZoomableWidget::paintEvent(QPaintEvent *) {
	QPainter painter(viewport());
	
	if(world.isEmpty()) {		
		paintEmptyContent(painter, viewport()->rect());		
	} else {
		QPoint topLeft = contentTopLeft();
		
		switch(_transformMode) {
		case Transform_HMirror:
			painter.translate(topLeft.x() + _zoomFactor * world.scaleFactor * (world.size.width() + world.offset.x()), topLeft.y() + _zoomFactor * world.scaleFactor * (world.size.height() + world.offset.y()));
			painter.scale(-_zoomFactor * world.scaleFactor, -_zoomFactor * world.scaleFactor);
			break;
		case Transform_VMirror:
			painter.translate(topLeft.x() - _zoomFactor * world.scaleFactor * world.offset.x(), topLeft.y() - _zoomFactor * world.scaleFactor * world.offset.y());
			painter.scale(_zoomFactor * world.scaleFactor, _zoomFactor * world.scaleFactor);
			break;		
		case Transform_90:
			painter.translate(topLeft.x() + _zoomFactor * world.scaleFactor * (world.size.height() + world.offset.y()), topLeft.y() + _zoomFactor * world.scaleFactor * (world.size.width() + world.offset.x()));
			painter.scale(_zoomFactor * world.scaleFactor, -_zoomFactor * world.scaleFactor);		
			painter.rotate(90);
			break;
		case Transform_90_HMirror:
			painter.translate(topLeft.x() - _zoomFactor * world.scaleFactor * world.offset.y(), topLeft.y() + _zoomFactor * world.scaleFactor * (world.size.width() + world.offset.x()));
			painter.scale(-_zoomFactor * world.scaleFactor, -_zoomFactor * world.scaleFactor);		
			painter.rotate(90);
			break;
		case Transform_90_VMirror:
			painter.translate(topLeft.x() + _zoomFactor * world.scaleFactor * (world.size.height() + world.offset.y()), topLeft.y() + _zoomFactor * world.scaleFactor * (world.size.width() + world.offset.x()));
			painter.scale(_zoomFactor * world.scaleFactor, _zoomFactor * world.scaleFactor);
			painter.rotate(90);
			break;
		case Transform_180:
			painter.translate(topLeft.x() + _zoomFactor * world.scaleFactor * (world.size.width() + world.offset.x()), topLeft.y() - _zoomFactor * world.scaleFactor * world.offset.y());
			painter.scale(-_zoomFactor * world.scaleFactor, _zoomFactor * world.scaleFactor);
			break;
		case Transform_270:
			painter.translate(topLeft.x() - _zoomFactor * world.scaleFactor * world.offset.y(), topLeft.y() - _zoomFactor * world.scaleFactor * world.offset.x());
			painter.scale(_zoomFactor * world.scaleFactor, -_zoomFactor * world.scaleFactor);
			painter.rotate(-90);
			break;
		
		case Transform_None:	
		default:
			painter.translate(topLeft.x() - _zoomFactor * world.scaleFactor * world.offset.x(), topLeft.y() + _zoomFactor * world.scaleFactor * (world.size.height() + world.offset.y()));
			painter.scale(_zoomFactor * world.scaleFactor, -_zoomFactor * world.scaleFactor);
		}
		
		paintContent(painter);
		
		painter.resetTransform();
		painter.setOpacity(1.0);
		if(_showOverlays) paintOverlays(painter, viewport()->rect());
	}
}

void ZoomableWidget::resizeEvent(QResizeEvent *event) {
	updateScrollBars();
	QAbstractScrollArea::resizeEvent(event);
}
void ZoomableWidget::wheelEvent(QWheelEvent *event) {
	if(world.isEmpty()) return;
	int delta = event->delta();
	if(delta < 0){
		delta = qMax(-delta, 120) / 120;
		setZoomFactor(_zoomFactor / (1 << delta), event->pos());
	}else{
		delta = qMax(delta, 120) / 120;
		setZoomFactor(_zoomFactor * (1 << delta), event->pos());
	}
}

void ZoomableWidget::mouseMoveEvent(QMouseEvent *event) {
	if(world.isEmpty()) return;
	
	emit mousePosChanged(contentToWorld(widgetToContent(event->pos())));	

	if(event->buttons() & Qt::MidButton){
		QPoint delta = panStartPos - event->pos();
		scrollBy(delta.x(), delta.y());
		panStartPos = event->pos();		
	}
	worldMouseMoveEvent(widgetToWorld(event->pos()), event->buttons());	
}
void ZoomableWidget::mousePressEvent(QMouseEvent *event) {
	if(world.isEmpty()) return;
	if(event->button() == Qt::MidButton) panStartPos = event->pos();		
	worldMousePressEvent(widgetToWorld(event->pos()), event->buttons(), event->button());	
}
void ZoomableWidget::mouseReleaseEvent(QMouseEvent *event) {
	if(world.isEmpty()) return;
	worldMouseReleaseEvent(widgetToWorld(event->pos()), event->buttons(), event->button());
}
	
void ZoomableWidget::setZoomFactor(qreal newZoomFactor, QPoint center) {
	// limit zoom factor from 1/8 to 16
	if(newZoomFactor < _minZoom) newZoomFactor = _minZoom;
	if(newZoomFactor > _maxZoom) newZoomFactor = _maxZoom;
	
	if(_zoomFactor != newZoomFactor){
		QPointF contentAtCenter = widgetToContent(center);
		
		_zoomFactor = newZoomFactor;
		if(_zoomInAction) _zoomInAction->setEnabled(_zoomFactor < _maxZoom);
		if(_zoomOutAction) _zoomOutAction->setEnabled(_zoomFactor > _minZoom);
		if(_zoomResetAction) _zoomResetAction->setEnabled(_zoomFactor != 1.0);
		
		updateScrollBars();
		
		QPoint deltaScroll = contentToWidget(contentAtCenter) - center;
		scrollBy(deltaScroll.x(), deltaScroll.y());
		emit zoomFactorChanged(_zoomFactor);
		viewport()->update();
	}	
}

QPoint ZoomableWidget::contentTopLeft() const {
	// translate control coordiantes to content coordinates
	QSize zoomedContentSize = QSizeF(_zoomFactor * contentSize).toSize();
	QSize viewportSize = viewport()->size();
	
	QPoint topLeft;	
	if(viewportSize.width() < zoomedContentSize.width()) topLeft.setX(-horizontalScrollBar()->value());
	else topLeft.setX((viewportSize.width() - zoomedContentSize.width()) / 2);
	if(viewportSize.height() < zoomedContentSize.height()) topLeft.setY(-verticalScrollBar()->value());
	else topLeft.setY((viewportSize.height() - zoomedContentSize.height()) / 2);
	
	return topLeft;	
}
QPointF ZoomableWidget::widgetToContent(const QPoint &pt) const {
    return QPointF(pt - contentTopLeft()) / _zoomFactor;	
}
QPoint ZoomableWidget::contentToWidget(const QPointF &pt) const {
	return contentTopLeft() + QPointF(pt * _zoomFactor).toPoint();	
}
QPointF ZoomableWidget::contentToWorld(const QPointF &pt) const {	
	switch(_transformMode) {
	case Transform_HMirror:
		return QPointF(world.size.width() + world.offset.x() - pt.x() * world.invScaleFactor, world.size.height() + world.offset.y() - pt.y() * world.invScaleFactor);
	case Transform_VMirror:
		return QPointF(world.offset.x() + pt.x() * world.invScaleFactor, world.offset.y() + pt.y() * world.invScaleFactor);
	case Transform_90:
		return QPoint(world.size.width() + world.offset.x() - pt.y() * world.invScaleFactor, world.size.height() + world.offset.y() - pt.x() * world.invScaleFactor);
	case Transform_90_HMirror:
		return QPoint(world.size.width() + world.offset.x() - pt.y() * world.invScaleFactor, world.offset.y() + pt.x() * world.invScaleFactor);
	case Transform_90_VMirror:
		return QPoint(world.offset.x() + pt.y() * world.invScaleFactor, world.size.height() + world.offset.y() - pt.x() * world.invScaleFactor);
	case Transform_180:
		return QPointF(world.offset.x() + world.size.width() - pt.x() * world.invScaleFactor, pt.y() * world.invScaleFactor + world.offset.y());
	case Transform_270:
		return QPointF(pt.y() * world.invScaleFactor + world.offset.x(), pt.x() * world.invScaleFactor + world.offset.y());
	case Transform_None:	
	default:
		return QPointF(world.offset.x() + pt.x() * world.invScaleFactor, world.size.height() + world.offset.y() - pt.y() * world.invScaleFactor);
	}
}
QPointF ZoomableWidget::worldToContent(const QPointF &pt) const {	
	switch(_transformMode) {
	case Transform_HMirror:
		return world.scaleFactor * (QPointF(world.size.width() + world.offset.x(), world.size.height() + world.offset.y()) - pt);
	case Transform_VMirror:
		return world.scaleFactor * (pt - world.offset);
	case Transform_90:
		return world.scaleFactor * QPointF(world.size.height() + world.offset.y() - pt.y(), world.size.width() + world.offset.x() - pt.x());
	case Transform_90_HMirror:
		return world.scaleFactor * QPointF(pt.y() - world.offset.y(),world.size.width() + world.offset.x() - pt.x());
	case Transform_90_VMirror:
		return world.scaleFactor * QPointF(world.size.height() + world.offset.y() - pt.y(), pt.x() - world.offset.x());
	case Transform_180:
		return world.scaleFactor * QPointF(world.size.width() + world.offset.x() - pt.x(), pt.y() - world.offset.y());
	case Transform_270:
		return world.scaleFactor * QPointF(pt.y() - world.offset.y(),pt.x() - world.offset.x());
	case Transform_None:
	default:
		return world.scaleFactor * QPointF(pt.x() - world.offset.x(), world.size.height() + world.offset.y() - pt.y());
	}
}

QPoint ZoomableWidget::worldToWidget(QPointF ptWorld) {
	return contentToWidget(worldToContent(ptWorld));	
}
QPointF ZoomableWidget::widgetToWorld(QPoint ptWidget) {
	return contentToWorld(widgetToContent(ptWidget));	
}

void ZoomableWidget::updateScrollBars() {
	QSize zoomedContentSize = QSizeF(_zoomFactor * contentSize).toSize();
	QSize viewportSize = viewport()->size();
	
   	QScrollBar *hsb = horizontalScrollBar();
	hsb->setRange(0, zoomedContentSize.width() - viewportSize.width());
	hsb->setPageStep(viewportSize.width());
	hsb->setSingleStep(qMax(1, (int)_zoomFactor));
	
	QScrollBar *vsb = verticalScrollBar();
	vsb->setRange(0, zoomedContentSize.height() - viewportSize.height());
	vsb->setPageStep(viewportSize.height());
	vsb->setSingleStep(qMax(1, (int)_zoomFactor));	
}
void ZoomableWidget::updateContent() {
	if(!world.isEmpty()) viewport()->update();
}

QActionGroup *ZoomableWidget::zoomActionGroup() {
	if(!_zoomActionGroup) _zoomActionGroup = new QActionGroup(this);
	return _zoomActionGroup;
}
QAction *ZoomableWidget::zoomInAction() {
	if(!_zoomInAction) {
		_zoomInAction = new QAction(QIcon(tr(":images/zoom_in.svg")), trUtf8("Zoom In"), this);
		_zoomInAction->setEnabled(_zoomFactor < _maxZoom);
		zoomActionGroup()->addAction(_zoomInAction);
		connect(_zoomInAction, SIGNAL(triggered(bool)), this, SLOT(zoomIn()));
	}
	return _zoomInAction;
}
QAction *ZoomableWidget::zoomOutAction() {
	if(!_zoomOutAction) {
		_zoomOutAction = new QAction(QIcon(tr(":images/zoom_out.svg")), trUtf8("Zoom Out"), this);
		_zoomOutAction->setEnabled(_zoomFactor > _minZoom);
		zoomActionGroup()->addAction(_zoomOutAction);
		connect(_zoomOutAction, SIGNAL(triggered(bool)), this, SLOT(zoomOut()));
	}
	return _zoomOutAction;
}
QAction *ZoomableWidget::zoomResetAction() {
	if(!_zoomResetAction) {
		_zoomResetAction = new QAction(QIcon(tr(":images/zoom_original.svg")), trUtf8("Reset zoom"), this);
		_zoomResetAction->setEnabled(_zoomFactor != 1.0);
		zoomActionGroup()->addAction(_zoomResetAction);
		connect(_zoomResetAction, SIGNAL(triggered(bool)), this, SLOT(resetZoom()));
	}
	return _zoomResetAction;
}

QActionGroup *ZoomableWidget::rotateActionGroup() {
	if(!_rotateActionGroup) {
		_rotateActionGroup = new QActionGroup(this);
		_rotateActionGroup->setExclusive(true);
		connect(_rotateActionGroup, SIGNAL(triggered(QAction *)), this, SLOT(changeRotationFromAction(QAction *)));
	}
	return _rotateActionGroup;
}
void ZoomableWidget::changeRotationFromAction(QAction *action) {
	if(action == _rotate0Action) setRotation(ZoomableWidget::Rotate_None);
	else if(action == _rotate90CWAction) setRotation(ZoomableWidget::Rotate_90CW);
	else if(action == _rotate90CCWAction) setRotation(ZoomableWidget::Rotate_90CCW);
	else if(action == _rotate180Action) setRotation(ZoomableWidget::Rotate_180);
}

QAction *ZoomableWidget::rotate0Action() {
	if(!_rotate0Action) {
		_rotate0Action = new QAction(QIcon(tr(":images/rotate_0.svg")), trUtf8("No Rotation"), this);
		_rotate0Action->setCheckable(true);
		rotateActionGroup()->addAction(_rotate0Action);		
		_rotate0Action->setChecked(_rotation == Rotate_None);		
	}
	return _rotate0Action;		
}
QAction *ZoomableWidget::rotate90CWAction() {
	if(!_rotate90CWAction) {
		_rotate90CWAction = new QAction(QIcon(tr(":images/rotate_270.svg")), trUtf8("Rotate 90° CW"), this);
		_rotate90CWAction->setCheckable(true);		
		rotateActionGroup()->addAction(_rotate90CWAction);
		_rotate90CWAction->setChecked(_rotation == Rotate_90CW);		
	}
	return _rotate90CWAction;		
}
QAction *ZoomableWidget::rotate90CCWAction() {
	if(!_rotate90CCWAction) {
		_rotate90CCWAction = new QAction(QIcon(tr(":images/rotate_90.svg")), trUtf8("Rotate 90° CCW"), this);
		_rotate90CCWAction->setCheckable(true);
		rotateActionGroup()->addAction(_rotate90CCWAction);
		_rotate90CCWAction->setChecked(_rotation == Rotate_90CCW);		
	}
	return _rotate90CCWAction;	
}
QAction *ZoomableWidget::rotate180Action() {
	if(!_rotate180Action) {
		_rotate180Action = new QAction(QIcon(tr(":images/rotate_180.svg")), trUtf8("Rotate 180°"), this);
		_rotate180Action->setCheckable(true);
		rotateActionGroup()->addAction(_rotate180Action);
		_rotate180Action->setChecked(_rotation == Rotate_180);
	}
	return _rotate180Action;		
}

QActionGroup *ZoomableWidget::mirrorActionGroup() {
	if(!_mirrorActionGroup) {
		_mirrorActionGroup = new QActionGroup(this);
		_mirrorActionGroup->setExclusive(false);
	}
	return _mirrorActionGroup;
}

QAction *ZoomableWidget::mirrorHAction() {
	if(!_mirrorHAction) {
		_mirrorHAction = new QAction(QIcon(tr(":images/mirror_horizontal.svg")), trUtf8("Horizontal mirroring"), this);
		_mirrorHAction->setCheckable(true);
		mirrorActionGroup()->addAction(_mirrorHAction);
		_mirrorHAction->setChecked(_mirrorMode & Mirror_Horizontal);
		connect(_mirrorHAction, SIGNAL(toggled(bool)), this, SLOT(setHMirror(bool)));
	}
	return _mirrorHAction;			
}
QAction *ZoomableWidget::mirrorVAction() {
	if(!_mirrorVAction) {
		_mirrorVAction = new QAction(QIcon(tr(":images/mirror_vertical.svg")), trUtf8("Vertical mirroring"), this);
		_mirrorVAction->setCheckable(true);
		mirrorActionGroup()->addAction(_mirrorVAction);
		_mirrorVAction->setChecked(_mirrorMode & Mirror_Vertical);
		connect(_mirrorVAction, SIGNAL(toggled(bool)), this, SLOT(setVMirror(bool)));
	}
	return _mirrorVAction;		
}

ZoomableWidget::TransformMode ZoomableWidget::calcTransformMode() {
	switch(_rotation) {
	case Rotate_90CCW:
		if((_mirrorMode & Mirror_Horizontal) && (_mirrorMode & Mirror_Vertical)) return Transform_270;
		else if(_mirrorMode & Mirror_Horizontal) return Transform_90_HMirror;
		else if(_mirrorMode & Mirror_Vertical) return Transform_90_VMirror;
		else return Transform_90;
	case Rotate_90CW:
		if((_mirrorMode & Mirror_Horizontal) && (_mirrorMode & Mirror_Vertical)) return Transform_90;
		else if(_mirrorMode & Mirror_Horizontal) return Transform_90_VMirror;
		else if(_mirrorMode & Mirror_Vertical) return Transform_90_HMirror;
		else return Transform_270;	
	case Rotate_180:
		if((_mirrorMode & Mirror_Horizontal) && (_mirrorMode & Mirror_Vertical)) return Transform_None;
		else if(_mirrorMode & Mirror_Horizontal) return Transform_VMirror;
		else if(_mirrorMode & Mirror_Vertical) return Transform_HMirror;
		else return Transform_180;		
	case Rotate_None:
	default:
		if((_mirrorMode & Mirror_Horizontal) && (_mirrorMode & Mirror_Vertical)) return Transform_180;
		else if(_mirrorMode & Mirror_Horizontal) return Transform_HMirror;
		else if(_mirrorMode & Mirror_Vertical) return Transform_VMirror;
		else return Transform_None;
	}
}

void ZoomableWidget::paintEmptyContent(QPainter &painter, const QRect &area) {
	QPalette pal = viewport()->palette();
	QColor bgColor = pal.color(QPalette::Background);
	
	QColor invColor(255 - bgColor.red() , 255 - bgColor.green(), 255 - bgColor.blue());
	
	painter.setPen(invColor);
	painter.setOpacity(0.2);
	QFont font = painter.font();
	font.setBold(true);
	font.setPixelSize(2 * qMin(area.height(), area.width()) / 3);
	painter.setFont(font);
	painter.drawText(area, Qt::AlignCenter, "?"); // big centered question mark	
}
