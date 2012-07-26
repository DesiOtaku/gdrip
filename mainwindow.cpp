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
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::openImage() {

    QString fileName = QFileDialog::getOpenFileName(this,tr("Open Image"),
                                                    QDir::homePath(),
                                                    tr("Image Files (*.png *.jpg *.bmp)"));
    m_original.load(fileName);
    m_current = m_original;

    ui->radioImageWidget->setImage(m_original);
    ui->histoWidget->setProcessImage(m_original);

    this->setWindowFilePath(fileName);
}

void MainWindow::showAbout() {
    AboutDialog dia(this);
    dia.exec();
}

void MainWindow::handleEqualize() {
    m_current = ImageProcessor::equalizeHistogram(m_current);
    ui->radioImageWidget->setImage(m_current);
    ui->histoWidget->setProcessImage(m_current);
}

void MainWindow::handleStartOver() {
    m_current = m_original;
    ui->radioImageWidget->setImage(m_original);
    ui->histoWidget->setProcessImage(m_original);
}
