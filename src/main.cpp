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

#include <QApplication>
#include "simmainwindow.h"

int main(int argc, char *argv[]){
	QApplication app(argc, argv);
	app.setOrganizationName("TUD");
	app.setOrganizationDomain("IFA");
	app.setApplicationName(QObject::tr("Raster Path Planner Simulator"));
	app.setApplicationVersion("V0.1.0");

	SimMainWindow mainWindow;
	mainWindow.show();
	return app.exec();
}
