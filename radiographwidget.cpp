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
#include <QPropertyAnimation>

#include "math.h"
#include "imageprocessor.h"

RadiographWidget::RadiographWidget(QWidget *parent) :
    QGraphicsView(parent),
    ui(new Ui::RadiographWidget) {
    ui->setupUi(this);
    setMouseTracking(true);
    m_BrightnessSet = 50;
    m_ContrastSet = 50;

    QGraphicsScene *scene = new QGraphicsScene(this);
    m_PixItem= new QGraphicsPixmapItem(0,scene);

    this->setScene(scene);
    this->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

    m_MJItem = new QGraphicsRectItem(m_PixItem->boundingRect(),m_PixItem);
    m_MJItem->setBrush(QBrush(Qt::green)); //if you actually see green, then there was
    //something wrong with the "setBrightness" function

    m_MJEffect = new QGraphicsOpacityEffect();
    m_MJEffect->setOpacity(0);
    m_MJItem->setGraphicsEffect(m_MJEffect);
    this->setDragMode(QGraphicsView::ScrollHandDrag);

    //Just making the distance line stuff ready so we can use it as soon as we are ready
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

void RadiographWidget::setImage(QImage img) {
    m_Original = img;
    m_NonContrastedImg = img;
    m_ContrastedImg = img;
    QPixmap pixmap;
    pixmap.convertFromImage(img,Qt::AutoColor);
    m_PixItem->setPixmap(pixmap);
    QRectF bounds = m_PixItem->boundingRect();
    m_PixItem->setTransformOriginPoint(bounds.width()/2,bounds.height()/2);
    m_MJItem->setRect(m_PixItem->boundingRect());
    newHistogram(ImageProcessor::findOccurrences(m_Original));

    foreach(QGraphicsLineItem *item, m_Marklines) {
        this->scene()->removeItem(item);
        delete item;
    }
    m_Marklines.clear();

    foreach(QGraphicsRectItem *item, m_Markdots) {
        this->scene()->removeItem(item);
        delete item;
    }
    m_Markdots.clear();


    QTransform trans;
    m_PixItem->setTransform(trans);
    this->resetMatrix();

    QPropertyAnimation *ani = new QPropertyAnimation(this);
    ani->setTargetObject(this);
    ani->setPropertyName("zoom");
    ani->setDuration(1000);
    ani->setStartValue(0);
    ani->setEndValue(50);
    QEasingCurve curve(QEasingCurve::OutElastic);
    curve.setAmplitude(1);
    ani->setEasingCurve(curve);
    ani->start(QAbstractAnimation::DeleteWhenStopped);
}

void RadiographWidget::setZoom(int newZoom) {
    float amount =  (newZoom/50.0);
    //m_PixItem->setScale(amount); //I have no clue why this moves the screen over center
    this->resetMatrix(); //don't worry, this doesn't actually show
    this->scale(amount,amount); //we do the whole scene because we want the markings
    //to scale as well. It also makes the distance calculation a lot easier
}

void RadiographWidget::setRotation(int angle) {
    m_PixItem->setRotation(angle);
}

void RadiographWidget::setBrightness(int amount) {
    //Lets abuse QGraphicsView and OpenGL to do the work on the GPU rather
    //than the CPU
    if(amount < 50) { //darken
        m_MJItem->setBrush(Qt::black);
        m_MJEffect->setOpacity(((50-amount) * 2) / 100.0);
    } else { //brighten
        m_MJItem->setBrush(Qt::white);
        m_MJEffect->setOpacity(((amount-50) * 2) / 100.0);
    }
    m_BrightnessSet = amount;
    updateHistogram();
}

void RadiographWidget::setContrast(int amount) {
    m_ContrastedImg = ImageProcessor::constrastImage(m_NonContrastedImg,amount);
    QPixmap pixmap;
    pixmap.convertFromImage(m_ContrastedImg,Qt::AutoColor);
    m_PixItem->setPixmap(pixmap);
    m_ContrastSet = amount;
    updateHistogram();
}

void RadiographWidget::invertImg() {
    m_NonContrastedImg = ImageProcessor::invertImage(m_NonContrastedImg);
    setContrast(m_ContrastSet);
}

void RadiographWidget::mirrorV() {
    QTransform trans;
    QRectF bounds = m_PixItem->boundingRect();
    trans.translate(bounds.width()/2,bounds.height()/2);
    trans.rotate(180,Qt::XAxis);
    trans.translate(bounds.width()/-2,bounds.height()/-2);
    m_PixItem->setTransform(trans,true);
}

void RadiographWidget::mirrorH() {
    QTransform trans;
    QRectF bounds = m_PixItem->boundingRect();
    trans.translate(bounds.width()/2,bounds.height()/2);
    trans.rotate(180,Qt::YAxis);
    trans.translate(bounds.width()/-2,bounds.height()/-2);
    m_PixItem->setTransform(trans,true);
}

void RadiographWidget::equalizeImg() {
    m_NonContrastedImg = ImageProcessor::equalizeHistogram(m_NonContrastedImg);
    setContrast(m_ContrastSet);
}

void RadiographWidget::strechHisto() {
    m_NonContrastedImg = ImageProcessor::spreadHistogram(m_NonContrastedImg);
    setContrast(m_ContrastSet);
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
        qreal ppmm = m_Original.dotsPerMeterX()/1000.0;
        qreal length = 1.0/(ppmm/ line.length());
        messageUpdate(tr("Length is: %1 mm").arg(length),3000);
    } else if(event->buttons() == Qt::LeftButton) {
        QGraphicsView::mouseMoveEvent(event);
    } else {
        QPointF scenepos = this->mapToScene(event->pos());
        QPointF po = m_PixItem->mapFromScene(scenepos);

        if((po.x() > 0) && (po.y() > 0)) {
            int offset=(int)  (((m_BrightnessSet -50)/50.0)*255);
            int valueHigh = qRed(m_ContrastedImg.pixel(po.toPoint())) + offset;
            valueHigh = qMax(0,valueHigh);
            valueHigh = qMin(255,valueHigh);
            pixelValueHighlighted(valueHigh);
            //messageUpdate(tr("Point value is: %1").arg(valueHigh),1000);
        }
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

void RadiographWidget::reset() {
    this->setImage(m_Original);
}

void RadiographWidget::addLine(QLine line, QColor color) {
    QGraphicsLineItem *newLine = new QGraphicsLineItem(line);
    newLine->setPen(QPen(QBrush(color),1,Qt::SolidLine,Qt::RoundCap));
    m_Marklines.append(newLine);
    this->scene()->addItem(newLine);
}

void RadiographWidget::addDot(QPoint point, QColor color) {
    QGraphicsRectItem *newItem = new QGraphicsRectItem(point.x(), point.y(), 1, 1);
    newItem->setBrush(QBrush(color));
    newItem->setPen(QPen(Qt::NoPen));
    this->scene()->addItem(newItem);
    m_Markdots.append(newItem);
}

void RadiographWidget::updateHistogram() {
    int offset=(int)  (((m_BrightnessSet -50)/50.0)*255); //because we cheated when we
    //did the brightness
    newHistogram(ImageProcessor::findOccurrences(m_ContrastedImg,offset));
}

QImage RadiographWidget::getOriginalImage() {
    return m_Original;
}

QImage RadiographWidget::getAlteredImage() {
    return m_ContrastedImg;
}

QImage RadiographWidget::getMarkedImage() {
    //TODO: add in the markings to the QImage
    QImage returnMe(m_ContrastedImg.size(),QImage::Format_ARGB32);

    for(int x=0;x<returnMe.width();x++) {
        for(int y=0;y<returnMe.height();y++) {
            int val = qRed(m_ContrastedImg.pixel(x,y));
            returnMe.setPixel(x,y,qRgb(val,val,val));
        }
    }

    foreach(QGraphicsRectItem *item, m_Markdots) {
        returnMe.setPixel((int)item->rect().topLeft().x(),
                          (int)item->rect().topLeft().y(),
                          item->brush().color().rgba());
    }

    return returnMe;
}
