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

#include "histowidget.h"
#include "imageprocessor.h"

#include <QPainter>
#include <QBrush>
#include <QDebug>
#include <QMouseEvent>

HistoWidget::HistoWidget(QWidget *parent) :
    QWidget(parent)
{
    setMouseTracking(true);
    m_HighlightValue=0;
}

/**
 * @brief HistoWidget::setProcessImage
 * Sets the image that the hisogram will be based on
 *
 * @param img
 * The image that the histogram should be based on
 */
void HistoWidget::setHistogram(QVector<float> values) {
    m_Occ = values;

    float max =0;
    for(int i=0;i<m_Occ.count();i++) {
        max = qMax(max,m_Occ.value(i));
    }

    float multi = 1.0 / max;

    for(int i=0;i<m_Occ.count();i++) {
        m_Occ.replace(i,m_Occ.value(i) * multi);
    }

    this->repaint();
}

void HistoWidget::highlightValue(int value) {
    m_HighlightValue = value;
    this->repaint();
}

/**
 * @brief HistoWidget::paintEvent
 * Overloading the paint event to draw the histogram
 */
void HistoWidget::paintEvent(QPaintEvent *) {
    QPainter p(this);
    QColor histoColor(70,70,40,150);
    p.setPen(histoColor);

    for(int i=0;i<256;i++) {
        float fraction =  m_Occ.value(i);
        int length =(int) this->height()*fraction;

        if(i == m_HighlightValue) {
            p.setPen(QColor(255,70,40));
            p.drawLine(i,this->height(),i,0);
            p.setPen(histoColor);
        }
        p.drawLine(i,this->height(),i,this->height()-length);
    }

}

/**
 * @brief HistoWidget::mouseMoveEvent
 * Update the tool tip to show the pixel value for the histogram
 * @param event
 */
void HistoWidget::mouseMoveEvent(QMouseEvent *event) {
    this->setToolTip(QString::number(event->x()));
}
