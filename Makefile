OBJS = main.o AffineMotion.o

CPPFLAGS = -O3 -fPIC 
CPPFLAGS += -I$(QGLVIEWER_DIR)/include
CPPFLAGS += -I$(QTDIR)/include -I$(QTDIR)/include/QtCore -I$(QTDIR)/include/QtWidgets -I$(QTDIR)/include/QtXml -I$(QTDIR)/include/QtOpenGL -I$(QTDIR)/include/QtGui
CPPFLAGS += -I$(SOFT_DIR)/MathLibs/AffineLib
CPPFLAGS += -I$(EIGEN_DIR)

CPPFLAGS += -I$(BOOST_DIR)/include

LIBS += -L$(QTDIR)/lib -lQt5Core -lQt5Xml -lQt5OpenGL -lQt5Widgets -lQt5Gui -lGL -lGLU
LIBS += -L$(QGLVIEWER_DIR)/lib -lQGLViewer

sam:$(OBJS)
	g++ -o sam $(OBJS) $(LIBS)

.o:.cpp
	g++ $(CPPFLAGS) $<

clean:
	\rm -rf *.o sam
