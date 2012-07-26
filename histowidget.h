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

#ifndef HISTOWIDGET_H
#define HISTOWIDGET_H

#include <QWidget>
#include <QImage>
#include <QVector>

/**
 * @brief The HistoWidget class
 * Creates a histogram of pixel values of the current image
 */
class HistoWidget : public QWidget
{
    Q_OBJECT
public:
    explicit HistoWidget(QWidget *parent = 0);
    void paintEvent(QPaintEvent *);
    void setProcessImage(QImage img);
    void mouseMoveEvent(QMouseEvent *event);

private:
    QVector<float> m_Occ;
    
};

#endif // HISTOWIDGET_H
