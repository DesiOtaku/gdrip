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

#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <QObject>
#include <QImage>
#include <QPainter>

class ImageProcessor : public QObject
{
    Q_OBJECT
public:
    explicit ImageProcessor(QObject *parent = 0);
    
    static QImage equalizeHistogram(QImage input);
    static QImage thresholdImage(QImage input, int cutoff);
    static QVector<int> findOcculsion(QImage input);
    static QVector<float> findOccurrences(QImage input);

    static void drawBezier(int p0x,int p0y,int p2x,int p2y,int p1x,int p1y, QPainter *img);
    
private:
    static int computeBezierSum(int p0x,int p0y,int p2x,int p2y,
                                int p1x,int p1y, int best, QImage img);
    static QVector<int> regValsBezier(int p0x,int p0y,int p2x,int p2y,
                                      int p1x,int p1y,int r, QImage img);
    static QVector<int> regionVals(int startX, int startY, int r, QImage img);

};

#endif // IMAGEPROCESSOR_H