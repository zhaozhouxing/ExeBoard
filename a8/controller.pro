#-------------------------------------------------
#
# Project created by QtCreator 2016-12-01T06:41:33
#
#-------------------------------------------------

QT       += core gui

TARGET = controller
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    ctrlapplication.cpp \
    memory.c \
    ccb.c \
    list.c \
    canItf.c \
    timer.c \
    log.c

HEADERS  += mainwindow.h \
    ctrlapplication.h

FORMS    += mainwindow.ui

OTHER_FILES +=

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/debug/ -lcommon
else:symbian: LIBS += -lcommon
else:unix: LIBS += -L$$PWD/ -lcommon

INCLUDEPATH += $$PWD/
DEPENDPATH += $$PWD/

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/release/common.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/debug/common.lib
else:unix:!symbian: PRE_TARGETDEPS += $$PWD/libcommon.a
