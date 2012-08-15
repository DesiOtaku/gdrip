#-------------------------------------------------
#
# Project created by QtCreator 2012-07-18T15:31:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = gdrip
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    radiographwidget.cpp \
    histowidget.cpp \
    aboutdialog.cpp \
    imageprocessor.cpp

HEADERS  += mainwindow.h \
    radiographwidget.h \
    histowidget.h \
    aboutdialog.h \
    imageprocessor.h

FORMS    += mainwindow.ui \
    radiographwidget.ui \
    aboutdialog.ui

RESOURCES += \
    icons.qrc
