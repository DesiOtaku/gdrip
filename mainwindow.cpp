/**********************************************************************
 * This file is part of the GNU Dental Radiograph Image Program, also
 * known as "gdrip."
 *
 * gdrip is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gdrip.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QDir>
#include <QDebug>
#include <QMimeData>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QUrl>
#include <QSettings>
#include <QFileInfo>
#include <QGraphicsDropShadowEffect>


#include "aboutdialog.h"
#include "imageprocessor.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->actionOpen,SIGNAL(triggered()),this,SLOT(openImage()));
    connect(ui->actionAbout,SIGNAL(triggered()),this,SLOT(showAbout()));
    connect(ui->zoomSlider,SIGNAL(valueChanged(int)),ui->radioImageWidget,SLOT(setZoom(int)));
    connect(ui->brightnessSlider,SIGNAL(valueChanged(int)),ui->radioImageWidget,SLOT(setBrightness(int)));
    connect(ui->rotationSlider,SIGNAL(valueChanged(int)),ui->radioImageWidget,SLOT(setRotation(int)));
    connect(ui->contrastSlider,SIGNAL(valueChanged(int)),ui->radioImageWidget,SLOT(setContrast(int)));
    connect(ui->actionEqualize_Histogram,SIGNAL(triggered()),ui->radioImageWidget,SLOT(equalizeImg()));
    connect(ui->actionStart_Over,SIGNAL(triggered()),this,SLOT(handleStartOver()));
    connect(ui->actionSave_Image,SIGNAL(triggered()),this,SLOT(handleSaveImage()));
    connect(ui->actionFind_Teeth,SIGNAL(triggered()),this,SLOT(handleFindTeeth()));
    connect(ui->actionMirrorVer,SIGNAL(triggered()),ui->radioImageWidget,SLOT(mirrorV()));
    connect(ui->actionMirror_Horizontally,SIGNAL(triggered()),ui->radioImageWidget,SLOT(mirrorH()));
    connect(ui->actionInvert_Image,SIGNAL(triggered()), ui->radioImageWidget,SLOT(invertImg()));
    connect(ui->actionStrech_Histogram,SIGNAL(triggered()),ui->radioImageWidget,SLOT(strechHisto()));
    connect(ui->radioImageWidget,SIGNAL(messageUpdate(QString,int)),this->statusBar(),SLOT(showMessage(QString,int)));

    connect(ui->radioImageWidget,SIGNAL(newHistogram(QVector<float>)),
            ui->histoWidget, SLOT(setHistogram(QVector<float>)));
    connect(ui->radioImageWidget,SIGNAL(pixelValueHighlighted(int)),
            ui->histoWidget, SLOT(highlightValue(int)));
    connect(ui->histoWidget, SIGNAL(valueSelected(int)), this, SLOT(handleHistoSelectPoint(int)));
    connect(ui->actionToggle_Marks, SIGNAL(toggled(bool)),this,SLOT(handleToggleMarks(bool)));
    connect(ui->actionFind_Caries_Reverse,SIGNAL(triggered()),this,SLOT(handleFindTeethReverse()));

    QSettings settings("tshah", "gdrip");
    restoreState(settings.value("windowState").toByteArray());
    restoreGeometry(settings.value("geometry").toByteArray());

    this->statusBar()->showMessage(tr("Ready"),3000);



}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event) {
    if (event->mimeData()->hasUrls()) { //TODO: make a const or something to keep the list of formats
        foreach (QUrl url, event->mimeData()->urls()) {
            if(url.path().endsWith(".png",Qt::CaseInsensitive) ||
                    url.path().endsWith(".jpg",Qt::CaseInsensitive) ||
                    url.path().endsWith(".jpeg",Qt::CaseInsensitive) ||
                    url.path().endsWith(".bmp",Qt::CaseInsensitive))
            event->acceptProposedAction();
        }
    }
}

void MainWindow::dropEvent(QDropEvent *event) {
    openImage(event->mimeData()->urls().at(0).path());
}

void MainWindow::closeEvent(QCloseEvent *event) {
    QSettings settings("tshah", "gdrip");
    settings.setValue("windowState", saveState());
    settings.setValue("geometry", saveGeometry());
    QMainWindow::closeEvent(event);
}

void MainWindow::openImage(QString fileName) {
    QImage startImg;
    startImg.load(fileName);
    ui->radioImageWidget->setImage(startImg);
    this->setWindowFilePath(fileName);
    this->statusBar()->showMessage(tr("Image \"%1\" has been opened").arg(fileName),3000);
    handleFindTeeth();
}

void MainWindow::openImage() {

    QSettings settings("tshah", "gdrip");
    QString startDir = settings.value("lastFolder",QDir::homePath()).toString();
    QString fileName = QFileDialog::getOpenFileName(this,tr("Open Image"),
                                                    startDir,
                                                    tr("Image Files (*.png *.jpg *.jpeg *.bmp)"));
    QFileInfo fileInstance(fileName);

    if(fileInstance.exists()) {
        openImage(fileName);
        settings.setValue("lastFolder", fileInstance.dir().absoluteFilePath(fileName));
    }
}

void MainWindow::showAbout() {
    AboutDialog dia(this);
    dia.exec();
}

void MainWindow::handleStartOver() {
    ui->zoomSlider->setValue(50);
    ui->brightnessSlider->setValue(50);
    ui->rotationSlider->setValue(0);
    ui->contrastSlider->setValue(50);

    ui->radioImageWidget->reset();
    this->statusBar()->showMessage(tr("Started over"),3000);
}


void MainWindow::handleSaveImage() {
    QString fileName = QFileDialog::getSaveFileName(this,tr("Save Image"),
                                                    QDir::homePath() + QDir::separator() + "img.png",
                                                    tr("Image Files (*.png *.jpg *.jpeg *.bmp)"));
    if(fileName.length() > 0) {
        ui->radioImageWidget->getMarkedImage().save(fileName);
    }
    this->statusBar()->showMessage(tr("Image has been saved to \"%1\"").arg(fileName),3000);
}


void MainWindow::handleFindTeeth() {
    this->statusBar()->showMessage(tr("Finding Teeth"),3000);
    QVector<QPair<QPoint, QColor> > drawMe = ImageProcessor::findTeeth(ui->radioImageWidget->getOriginalImage());
    ui->radioImageWidget->clearMarks();
    this->statusBar()->showMessage(tr("Found Teeth"),3000);
    for(int i=0;i<drawMe.count();i++) { //can't use foreach because C++ macros suck and the compiler will only complain about it
        QPair<QPoint, QColor> item = drawMe.at(i);
        ui->radioImageWidget->addDot(item.first,item.second);
    }
    this->statusBar()->showMessage(tr("Drawing Teeth"),3000);
}

void MainWindow::handleFindTeethReverse() {

    QImage img = ui->radioImageWidget->getOriginalImage().mirrored(true,false);

    ui->radioImageWidget->mirrorH();

    this->statusBar()->showMessage(tr("Finding Teeth"),3000);
    QVector<QPair<QPoint, QColor> > drawMe = ImageProcessor::findTeeth(img);
    ui->radioImageWidget->clearMarks();
    this->statusBar()->showMessage(tr("Found Teeth"),3000);
    for(int i=0;i<drawMe.count();i++) { //can't use foreach because C++ macros suck and the compiler will only complain about it
        QPair<QPoint, QColor> item = drawMe.at(i);
        ui->radioImageWidget->addDot(item.first,item.second);
    }
    this->statusBar()->showMessage(tr("Drawing Teeth"),3000);
}

void MainWindow::handleHistoSelectPoint(int valueSelected) {
    ui->radioImageWidget->clearMarks();
    QImage lookAt = ui->radioImageWidget->getAlteredImage();
    for(int x=0;x<lookAt.width();x++) {
        for(int y=0;y<lookAt.height();y++) {
            int value = qRed(lookAt.pixel(x,y));
            if(value == valueSelected) {
                ui->radioImageWidget->addDot(QPoint(x,y),QColor(0,0,255));
            }
        }
    }

}

void MainWindow::handleToggleMarks(bool showMarks) {
    ui->radioImageWidget->toggleMarks(showMarks);
}
