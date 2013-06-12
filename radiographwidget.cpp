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
#include <QUrl>
#include <QGraphicsDropShadowEffect>

#include "math.h"
#include "imageprocessor.h"
#include "mainwindow.h"

RadiographWidget::RadiographWidget(QWidget *parent) :
    QGraphicsView(parent),
    ui(new Ui::RadiographWidget) {
    ui->setupUi(this);
    setMouseTracking(true);
    m_MouseStatus = MOUSE_HOVER;
    m_BrightnessSet = 50;
    m_ContrastSet = 50;
    m_LastRotate =0;

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

    m_CrossEndItem = new QGraphicsEllipseItem(-5,-5,10,10,0,scene);
    m_CrossEndItem->setBrush(QBrush(Qt::blue));
    m_CrossEndItem->setVisible(false);
    m_CrossEndItem->setOpacity(0.6);


    m_CurveCrossStartItem  = new QGraphicsEllipseItem(-8,-8,10,10,0,scene);
    m_CurveCrossStartItem->setBrush(QBrush(Qt::red));
    m_CurveCrossStartItem->setVisible(false);
    m_CurveCrossStartItem->setOpacity(0.6);

    m_DistanceLineItem = new QGraphicsLineItem(0,scene);
    m_DistanceLineItem->setVisible(false);
    m_DistanceLineItem->setPen(QPen(QBrush(QColor(0,0,255,100)),5,Qt::DotLine,Qt::RoundCap));

    m_DistanceTextItem = new QGraphicsTextItem(0,scene);
    m_DistanceTextItem->setVisible(false);
    m_DistanceTextItem->setPlainText("0.00");
    m_DistanceTextItem->setDefaultTextColor(QColor(0,0,255));
    QFont defFont = m_DistanceTextItem->font();
    defFont.setBold(true);
    defFont.setPointSize(12);
    m_DistanceTextItem->setFont(defFont);

    QGraphicsDropShadowEffect *shadow = new QGraphicsDropShadowEffect(this);
    shadow->setBlurRadius(50);
    shadow->setColor(QColor(250,250,250));
    shadow->setOffset(QPointF(0,0));
    m_DistanceTextItem->setGraphicsEffect(shadow);

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

    clearMarks();

    m_CrossEndItem->setVisible(false);
    m_CrossStartItem->setVisible(false);
    m_DistanceLineItem->setVisible(false);
    m_DistanceTextItem->setVisible(false);


    QTransform trans;
    m_PixItem->setTransform(trans);
    this->resetMatrix();
    this->centerOn(m_PixItem);

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

void RadiographWidget::clearMarks() {
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
}

void RadiographWidget::setZoom(int newZoom) {
    float amount =  (newZoom/50.0);
    //m_PixItem->setScale(amount); //I have no clue why this moves the screen over center
    this->resetMatrix(); //don't worry, this doesn't actually show
    this->scale(amount,amount); //we do the whole scene because we want the markings
    //to scale as well. It also makes the distance calculation a lot easier

    //Because we reset the matrix, we have to also redo the rotation
    int oldRot = m_LastRotate;
    m_LastRotate =0;
    setRotation(oldRot);
}

void RadiographWidget::setRotation(int angle) {
    int diff = angle - m_LastRotate;
    this->rotate(diff);
    m_LastRotate = angle;
    //m_PixItem->setRotation(angle);
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
    //Not doing a swtich statement in case I want to use other factors
    if(m_MouseStatus == MOUSE_TRANS) {
        QPoint diff = event->pos() - m_mouseStartPoint;
        QTransform trans;
        QRectF bounds = m_PixItem->boundingRect();
        trans.translate(bounds.width()/2,bounds.height()/2);
        trans.rotate(diff.x() / 10.0,Qt::YAxis);
        trans.rotate(diff.y() / 10.0,Qt::XAxis);
        trans.translate(bounds.width()/-2,bounds.height()/-2);
        m_PixItem->setTransform(trans,true);
        m_mouseStartPoint = event->pos();
    } else if(m_MouseStatus == MOUSE_STR_DIST) {
        QLineF line(m_CrossStartItem->pos(),this->mapToScene(event->pos()));
        m_DistanceLineItem->setLine(line);
        qreal ppmm = m_Original.dotsPerMeterX()/1000.0;
        qreal length = 1.0/(ppmm/ line.length());
        m_DistanceTextItem->setPlainText(tr("%1 mm").arg(length));
        m_DistanceTextItem->setPos(line.p2());
        m_DistanceTextItem->setVisible(true);
        //messageUpdate(tr("Length is: %1 mm").arg(length),3000);
    } else if(event->buttons() == MOUSE_MOVE) {
        QGraphicsView::mouseMoveEvent(event);
    } else if(m_MouseStatus == MOUSE_CUR_DIST) {
        QPointF lastPoint = m_CurveDistanceLineItems.last()->line().p2();
        QPointF newPoint = this->mapToScene(event->pos());
        QLineF newLine(lastPoint,newPoint);
        QGraphicsLineItem *newLineItem = new QGraphicsLineItem(newLine);
        newLineItem->setPen(QPen(QBrush(QColor(255,0,0,100)),5,Qt::SolidLine,Qt::RoundCap));
        this->scene()->addItem(newLineItem);
        m_CurveDistanceLineItems.append(newLineItem);

        qreal sumDistance =0;
        foreach(QGraphicsLineItem *addItem,m_CurveDistanceLineItems) {
            sumDistance+= addItem->line().length();
        }

        qreal ppmm = m_Original.dotsPerMeterX()/1000.0;
        qreal length = 1.0/(ppmm/ sumDistance);
        messageUpdate(tr("Length is: %1 mm").arg(length),3000);

    } else { //MOUSE_HOVER
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
        m_MouseStatus = MOUSE_TRANS;
        m_mouseStartPoint = event->pos();
        this->viewport()->setCursor(QCursor(Qt::SizeAllCursor));
    } else if(event->buttons() == Qt::RightButton) {
        m_MouseStatus = MOUSE_STR_DIST;
        m_CrossStartItem->setPos(this->mapToScene(event->pos()));
        m_CrossStartItem->setVisible(true);
        m_CrossEndItem->setVisible(false);
        this->viewport()->setCursor(QCursor(Qt::CrossCursor));
        QLineF line(m_CrossStartItem->pos(),m_CrossStartItem->pos());
        m_DistanceLineItem->setLine(line);
        m_DistanceLineItem->setVisible(true);
        m_DistanceTextItem->setVisible(false); //in case the user just right clicks and doesn't move mouse
    } else if(m_MouseStatus == MOUSE_SEL_PT) {
        QPointF scenepos = this->mapToScene(event->pos());
        QPointF po = m_PixItem->mapFromScene(scenepos);
        emit pointSelected(po.toPoint());
        m_MouseStatus = MOUSE_HOVER;
    } else { //left click
        QGraphicsView::mousePressEvent(event);
        m_MouseStatus =MOUSE_MOVE;
    }
}

void RadiographWidget::mouseReleaseEvent(QMouseEvent *event) {
    if(m_MouseStatus == MOUSE_STR_DIST) {
        m_CrossEndItem->setPos(this->mapToScene(event->pos()));
        m_CrossEndItem->setVisible(true);
        QPropertyAnimation *scaleAni = new QPropertyAnimation(m_DistanceTextItem,"scale",this);
        scaleAni->setDuration(1000);
        scaleAni->setEasingCurve(QEasingCurve::OutElastic);
        scaleAni->setStartValue(.5);
        scaleAni->setEndValue(1);
        scaleAni->start(QAbstractAnimation::DeleteWhenStopped);
    }
    m_MouseStatus = MOUSE_HOVER;
    this->viewport()->setCursor(QCursor(Qt::OpenHandCursor));


    //m_CrossStartItem->setVisible(false);
    //m_DistanceLineItem->setVisible(false);
    QGraphicsView::mouseReleaseEvent(event);
}

void RadiographWidget::keyPressEvent(QKeyEvent *event) {
    if((event->key() == Qt::Key_C) && (!event->isAutoRepeat())) {
        m_MouseStatus = MOUSE_CUR_DIST;
        m_mouseCurveStartPoint = QPoint(); //it will be null
        QPoint widgetPoint = this->mapFromGlobal(QCursor::pos());
        m_CurveCrossStartItem->setPos(this->mapToScene(widgetPoint));
        m_CurveCrossStartItem->setVisible(true);
        this->viewport()->setCursor(QCursor(Qt::CrossCursor));
        QLineF firstLine(m_CurveCrossStartItem->pos(),m_CurveCrossStartItem->pos());
        QGraphicsLineItem *firstLineItem = new QGraphicsLineItem(firstLine);
        firstLineItem->setPen(QPen(QBrush(QColor(255,0,0,100)),5,Qt::SolidLine,Qt::RoundCap));
        m_CurveDistanceLineItems.append(firstLineItem);
        this->scene()->addItem(firstLineItem);

        //qDebug()<<
    }
    QGraphicsView::keyPressEvent(event);
}

void RadiographWidget::keyReleaseEvent(QKeyEvent *event) {
    if((event->key() == Qt::Key_C) && (!event->isAutoRepeat())) {
        m_CurveCrossStartItem->setVisible(false);
        this->viewport()->setCursor(QCursor(Qt::OpenHandCursor));

        foreach(QGraphicsLineItem *item,m_CurveDistanceLineItems) {
            this->scene()->removeItem(item);
            delete item;
        }
        m_CurveDistanceLineItems.clear();
    }
    QGraphicsView::keyReleaseEvent(event);
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

void RadiographWidget::selectPoint() {
    m_MouseStatus = MOUSE_SEL_PT;
    this->viewport()->setCursor(QCursor(Qt::CrossCursor));
}
