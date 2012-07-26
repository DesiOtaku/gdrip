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
    
public:
    explicit RadiographWidget(QWidget *parent = 0);
    void setImage(QImage img);
    ~RadiographWidget();
    
public slots:
    void setZoom(int newZoom);

private:
    Ui::RadiographWidget *ui;
    int m_zoom;

    QPixmap m_pix;
};

#endif // RADIOGRAPHWIDGET_H
