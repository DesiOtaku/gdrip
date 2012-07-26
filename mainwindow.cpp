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
    connect(ui->actionEqualize_Histogram,SIGNAL(triggered()),
            this,SLOT(handleEqualize()));
    connect(ui->actionStart_Over,SIGNAL(triggered()),
            this,SLOT(handleStartOver()));
    connect(ui->actionDraw_Occulsion,SIGNAL(triggered()),
            this,SLOT(handleDrawOcc()));

    this->statusBar()->showMessage(tr("Ready"),3000);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event) {
    if (event->mimeData()->hasUrls()) {
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

void MainWindow::openImage(QString fileName) {
    m_original.load(fileName);
    m_current = m_original;

    ui->radioImageWidget->setImage(m_original);
    ui->histoWidget->setProcessImage(m_original);

    this->setWindowFilePath(fileName);

    this->statusBar()->showMessage(tr("Image \"%1\" has been opened").arg(fileName),3000);
}

void MainWindow::openImage() {

    QString fileName = QFileDialog::getOpenFileName(this,tr("Open Image"),
                                                    QDir::homePath(),
                                                    tr("Image Files (*.png *.jpg *.jpeg *.bmp)"));
    if(QFile::exists(fileName)) {
        openImage(fileName);
    }
}

void MainWindow::showAbout() {
    AboutDialog dia(this);
    dia.exec();
}

void MainWindow::handleEqualize() {
    m_current = ImageProcessor::equalizeHistogram(m_current);
    ui->radioImageWidget->setImage(m_current);
    ui->histoWidget->setProcessImage(m_current);
    this->statusBar()->showMessage(tr("Done with histogram equalization"),3000);
}

void MainWindow::handleStartOver() {
    m_current = m_original;
    ui->radioImageWidget->setImage(m_original);
    ui->histoWidget->setProcessImage(m_original);
    this->statusBar()->showMessage(tr("Started over"),3000);
}

void MainWindow::handleDrawOcc() {
    m_current = ImageProcessor::drawOcculsion(m_current);
    ui->radioImageWidget->setImage(m_current);
    ui->histoWidget->setProcessImage(m_current);
    this->statusBar()->showMessage(tr("Done finding occlusion"),3000);
}
