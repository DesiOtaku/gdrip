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
#include <QGraphicsRectItem>
#include <QRectF>
#include <QDragMoveEvent>
#include <QTransform>
#include <QBitmap>

#include "math.h"

RadiographWidget::RadiographWidget(QWidget *parent) :
    QGraphicsView(parent),
    ui(new Ui::RadiographWidget) {
    ui->setupUi(this);
    m_Rotation = 0;

    QGraphicsScene *scene = new QGraphicsScene(this);
    m_PixItem= new QGraphicsPixmapItem(0,scene);

    this->setScene(scene);
    this->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

    m_MJItem = new QGraphicsRectItem(m_PixItem->boundingRect(),m_PixItem);
    m_MJItem->setBrush(QBrush(Qt::green));

    m_MJEffect = new QGraphicsOpacityEffect();
    m_MJEffect->setOpacity(0);
    m_MJItem->setGraphicsEffect(m_MJEffect);
    this->setDragMode(QGraphicsView::ScrollHandDrag);

    m_CrossStartItem = new QGraphicsEllipseItem(-5,-5,10,10,0,scene);
    m_CrossStartItem->setBrush(QBrush(Qt::blue));
    m_CrossStartItem->setVisible(false);
    m_CrossStartItem->setOpacity(0.6);

    m_DistanceLineItem = new QGraphicsLineItem(0,scene);
    m_DistanceLineItem->setVisible(false);
    m_DistanceLineItem->setPen(QPen(QBrush(QColor(0,0,255,100)),5,Qt::DotLine,Qt::RoundCap));
}

RadiographWidget::~RadiographWidget() {
    delete ui;
}

void RadiographWidget::setZoom(int newZoom) {
    float amount =  (newZoom/50.0);
    //m_PixItem->setScale(amount); //I have no clue why this moves the screen over center
    this->resetMatrix();
    this->scale(amount,amount);
}

void RadiographWidget::setRotation(int angle) {
    m_Rotation = angle;
    m_PixItem->setRotation(angle);
}

void RadiographWidget::setBrightness(int amount) {
    if(amount < 50) { //darken
        m_MJItem->setBrush(Qt::black);
        m_MJEffect->setOpacity(((50-amount) * 2) / 100.0);
    } else { //brighten
        m_MJItem->setBrush(Qt::white);
        m_MJEffect->setOpacity(((amount-50) * 2) / 100.0);
    }
}

void RadiographWidget::setImage(QImage img) {
    QPixmap pixmap;
    pixmap.convertFromImage(img,Qt::ColorOnly);
    m_PixItem->setPixmap(pixmap);
    QRectF bounds = m_PixItem->boundingRect();
    m_PixItem->setTransformOriginPoint(bounds.width()/2,bounds.height()/2);
    m_MJItem->setRect(m_PixItem->boundingRect());
}

void RadiographWidget::mouseMoveEvent(QMouseEvent *event) {
    if(event->buttons() == Qt::MiddleButton) {
        QPoint diff = event->pos() - m_mouseStartPoint;
        QTransform trans;
        QRectF bounds = m_PixItem->boundingRect();
        trans.translate(bounds.width()/2,bounds.height()/2);
        trans.rotate(diff.x() / 10.0,Qt::YAxis);
        trans.rotate(diff.y() / 10.0,Qt::XAxis);
        trans.translate(bounds.width()/-2,bounds.height()/-2);
        m_PixItem->setTransform(trans,true);
        m_mouseStartPoint = event->pos();

    } else if(event->buttons() == Qt::RightButton) {
        QLineF line(m_CrossStartItem->pos(),this->mapToScene(event->pos()));
        m_DistanceLineItem->setLine(line);
        messageUpdate(tr("Length is: %1 pixels").arg(line.length()),3000);
    } else {
        QGraphicsView::mouseMoveEvent(event);
    }
}

void RadiographWidget::mousePressEvent(QMouseEvent *event) {
    if(event->buttons() == Qt::MiddleButton) {
        m_mouseStartPoint = event->pos();
        this->viewport()->setCursor(QCursor(Qt::SizeAllCursor));
    } else if(event->buttons() == Qt::RightButton) {
        m_CrossStartItem->setPos(this->mapToScene(event->pos()));
        m_CrossStartItem->setVisible(true);
        this->viewport()->setCursor(QCursor(Qt::CrossCursor));
        QLineF line(m_CrossStartItem->pos(),m_CrossStartItem->pos());
        m_DistanceLineItem->setLine(line);
        m_DistanceLineItem->setVisible(true);
    } else {
        QGraphicsView::mousePressEvent(event);
    }
}

void RadiographWidget::mouseReleaseEvent(QMouseEvent *event) {
    this->viewport()->setCursor(QCursor(Qt::OpenHandCursor));
    m_CrossStartItem->setVisible(false);
    m_DistanceLineItem->setVisible(false);
    QGraphicsView::mouseReleaseEvent(event);
}

void RadiographWidget::resetView() {
    QTransform trans;
    m_PixItem->setTransform(trans);
    this->resetMatrix();
}
