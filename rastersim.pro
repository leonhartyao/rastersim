TEMPLATE = app
TARGET = rastersim
QT += xml svg


#CONFIG = release

DEPENDPATH += .
INCLUDEPATH += .

unix:DESTDIR = bin_unix
win32:DESTDIR = bin_win

unix:MOC_DIR = tmp_unix/
win32:MOC_DIR = tmp_win/
unix:OBJECTS_DIR = tmp_unix/
win32:OBJECTS_DIR = tmp_win/
unix:RCC_DIR = tmp_unix/
win32:RCC_DIR = tmp_win/

unix:MAKEFILE = Makefile
win32:MAKEFILE = Makefile.win
QMAKE_CXXFLAGS += -g

# Input
HEADERS +=  src/data.h \
			src/robot.h \
			src/abstractplanner.h \
			src/astarplanner.h \
			src/dstarplanner.h \
			src/fdstarplanner.h \
			src/dstarliteplanner.h \
			src/simmainwindow.h \
			src/simwidget.h \
			src/zoomablewidget.h \
			src/visualizationwidget.h \
			src/flowlayout.h \
			src/rlcpens.h


SOURCES += 	src/main.cpp \
			src/data.cpp \
			src/robot.cpp \
			src/abstractplanner.cpp \
			src/astarplanner.cpp \
			src/dstarplanner.cpp \
			src/fdstarplanner.cpp \
			src/dstarliteplanner.cpp \
			src/zoomablewidget.cpp \
			src/visualizationwidget.cpp \
			src/flowlayout.cpp \
			src/simwidget.cpp \
			src/simmainwindow.cpp \
			src/rlcpens.cpp

RESOURCES += rastersim.qrc

#RC_FILE = res/appicon.rc
