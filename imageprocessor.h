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
#include "radiographwidget.h"

/**
 * @brief The ImageProcessor class
 * A singleton class that does a lot of the actual image processing
 */
class ImageProcessor : public QObject
{
    Q_OBJECT
public:
    explicit ImageProcessor(QObject *parent = 0);
    
    //Basic image editing
    static QImage equalizeHistogram(QImage input);
    static QImage brightenImage(QImage original, int amount); //not really used.. but kept just in case
    static QImage constrastImage(QImage original, int amount);
    static QImage invertImage(QImage input);
    static QImage spreadHistogram(QImage input); //not really used.. but kept just in case
    static QImage mirrorVertically(QImage input); //not really used.. but kept just in case
    static QImage mirrorHorizontally(QImage input); //not really used.. but kept just in case

    //Quick helpers
    static QVector<float> findOccurrences(QImage input, int offset=0);

    //Research functions
    static QImage drawOcculsion(QImage input);
    static QImage findBackground(QImage input);
    static QVector<QVariant> findTeeth(QImage input);
    static QVector<QVariant> findPulp(QImage input, QPoint startingPoint);

    
private:
    static QImage thresholdImage(QImage input, int cutoff);
    static void drawBezier(int p0x,int p0y,int p2x,int p2y,int p1x,int p1y, QPainter *img);
    static QVector<int> findOcculsion(QImage input);
    static QVector<QPoint> findOcculsionFaster(QImage input);
    static float calculateCenterValue(QImage input,int seeX,int seeY);
    static QVector<QLine> findEnamel(QImage input, QVector<QPoint> points, int cutOff);
    static QVector<QPoint> findOutline(QImage input, int cutoff, QPoint leftOcc, QPoint rightOcc);
    static qreal findStdevArea(QImage input, QPoint center, int radius);
    static QVector<QPoint> findSameX(QPoint needle, QVector<QPoint> haystack);
    static qreal calcVerticalConstrast(QImage input, QPoint center, int radius);
    static QVector<QPoint> findInterProximal(QImage input, QVector<QPoint> occPoints,QVector<QPoint> outlinePoints, int cutOff);
    static QVector<QVector<QPoint> > groupPoints(QVector<QPoint> points, int width, int height);
    static void resetMatrix(int** map, int previousValue, int newValue, int width, int height);
    static void mergeNeighbors(int** map, int x, int y,int width, int height);
    static void drawBezierDer(int p0x,int p0y,int p2x,int p2y,
                                int p1x,int p1y, int stDev, QPainter *input);
    static int computeBezierSum(int p0x,int p0y,int p2x,int p2y,
                                int p1x,int p1y, int best, QImage img);
    static QVector<int> regValsBezier(int p0x,int p0y,int p2x,int p2y,
                                      int p1x,int p1y,int r, QImage img);
    static QVector<int> regionVals(int startX, int startY, int r, QImage img);

};

#endif // IMAGEPROCESSOR_H
