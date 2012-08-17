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

#include "radiographwidget.h"
#include "ui_radiographwidget.h"

#include <QDebug>
#include <QColor>
#include <QPainter>
#include <QProgressDialog>
#include <QGraphicsPixmapItem>
#include <QBrush>

#include "math.h"

RadiographWidget::RadiographWidget(QWidget *parent) :
    QGraphicsView(parent),
    ui(new Ui::RadiographWidget)
{
    ui->setupUi(this);
    m_Rotation = 0;
}

RadiographWidget::~RadiographWidget()
{
    delete ui;
}

void RadiographWidget::setZoom(int newZoom) {
    float amount =  (newZoom/50.0);
    m_Item->setScale(amount);
}

void RadiographWidget::setRotation(int angle) {
    m_Rotation = angle;
    m_Item->setRotation(angle);
}

void RadiographWidget::setImage(QImage img) {
    QPixmap pixmap;
    pixmap.convertFromImage(img,Qt::ColorOnly);
    ui->label->setText("");

    QGraphicsScene *scene = new QGraphicsScene(this);
    m_Item=scene->addPixmap(pixmap);
    QRectF bounds = m_Item->boundingRect();
    m_Item->setTransformOriginPoint(bounds.width()/2,bounds.height()/2);
    this->setScene(scene);
    this->setInteractive(true);
    this->setRenderHints(QPainter::HighQualityAntialiasing | QPainter::SmoothPixmapTransform);
    this->setDragMode(QGraphicsView::ScrollHandDrag);
    setRotation(m_Rotation);
}
