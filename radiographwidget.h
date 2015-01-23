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


#ifndef RADIOGRAPHWIDGET_H
#define RADIOGRAPHWIDGET_H

#include <QWidget>
#include <QPixmap>
#include <QGraphicsView>
#include <QGraphicsOpacityEffect>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>

namespace Ui {
class RadiographWidget;
}

/**
 * @brief The RadiographWidget class
 * Displays the current radiograph the user is working on
 */
class RadiographWidget : public QGraphicsView
{
    Q_OBJECT
    Q_PROPERTY (float zoom WRITE setZoom)
    
public:
    explicit RadiographWidget(QWidget *parent = 0);
    ~RadiographWidget();

    QImage getOriginalImage();
    QImage getAlteredImage();
    QImage getMarkedImage();

    void setImage(QImage img);
    void clearMarks();
    void toggleMarks(bool showMarks);

    //When doing the anaylsis, it would be nice to have this added in
    void addLine(QLine line, QColor color);
    void addCircle(QPoint point, float radius, QColor color);
    void addDot(QPoint point, QColor color);


    //When the user does stuff with the mouse
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

    //When the user smacks something on the keyboard
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

public slots:
    void setZoom(int newZoom);
    void setRotation(int angle);
    void setBrightness(int amount);
    void setContrast(int amount);
    void invertImg();
    void equalizeImg();
    void strechHisto();
    void mirrorV();
    void mirrorH();
    void selectPoint();
    void reset();


signals:
    void messageUpdate(QString message,int timeout);
    void newHistogram(QVector<float> values);
    void pixelValueHighlighted(int value);
    void pointSelected(QPoint selectedPoint);


private:
    Ui::RadiographWidget *ui;

    //Keep track of what the mouse is doing
    enum MouseStatus {MOUSE_HOVER, MOUSE_MOVE, MOUSE_TRANS, MOUSE_STR_DIST,MOUSE_CUR_DIST, MOUSE_SEL_PT};
    MouseStatus m_MouseStatus;

    //The image itself
    QImage m_Original;
    QImage m_NonContrastedImg;
    QImage m_ContrastedImg;
    QGraphicsPixmapItem *m_PixItem;

    //Keeping track of user input when we do crazy things
    int m_BrightnessSet;
    int m_ContrastSet;
    int m_LastRotate;

    //Relating to single measurement
    QGraphicsEllipseItem *m_CrossStartItem;
    QGraphicsEllipseItem *m_CrossEndItem;
    QGraphicsLineItem *m_DistanceLineItem;
    QGraphicsTextItem *m_DistanceTextItem;
    QPoint m_mouseStartPoint;

    //Relating to a single curved measurement
    QGraphicsEllipseItem *m_CurveCrossStartItem;
    QVector<QGraphicsLineItem *> m_CurveDistanceLineItems;
    QGraphicsTextItem *m_CurveDistanceTextItem;
    QPoint m_mouseCurveStartPoint;

    //TODO: Angle measurement + line displacement

    //Brightness adjustment (named after a famous pop star)
    QGraphicsRectItem *m_MJItem;
    QGraphicsOpacityEffect *m_MJEffect;

    //Line and circle markings
    QVector<QGraphicsLineItem *> m_Marklines;
    QVector<QGraphicsEllipseItem *> m_Markcircles;
    QVector<QGraphicsRectItem *> m_Markdots;

    //Private functions
    void updateHistogram();


};

#endif // RADIOGRAPHWIDGET_H
