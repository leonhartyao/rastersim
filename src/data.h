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

#ifndef DATA_H
#define DATA_H

#include <QPointF>
#include <cmath>
#include <QVector>

class Pose2D {
public:
	Pose2D(): _pos(0.0, 0.0), _angle(0.0) { }
	Pose2D(const QPointF &pos): _pos(pos), _angle(0.0) { }
	Pose2D(const QPointF &pos, qreal angle): _pos(pos), _angle(angle) { }
	Pose2D(qreal x, qreal y, qreal angle): _pos(x, y), _angle(angle) { }
	
	const QPointF &pos() const { return _pos; }
	qreal angle() const { return _angle; }
	qreal x() const { return _pos.x(); }
	qreal y() const { return _pos.y(); }
	
	void setPos(const QPointF &pos) { _pos = pos; }
	void setPos(qreal x, qreal y) { _pos = QPointF(x, y); }
	void setAngle(qreal angle) { _angle = angle; }
	void setPose(const QPointF &pos, qreal angle) { _pos = pos; _angle = angle; }
	void setPose(qreal x, qreal y, qreal angle) { _pos = QPointF(x, y); _angle = angle; }
	
	bool isValid() const { return (!isnan(_pos.x()) && !isnan(_pos.y()) && !isnan(_angle)); }
	
	static const Pose2D &invalid() { return _invalid; }
	
private:
	QPointF _pos;
	qreal _angle;
	static Pose2D _invalid;
};

typedef QVector<QPointF> Path;

#endif // DATA_H

