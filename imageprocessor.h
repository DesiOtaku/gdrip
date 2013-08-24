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
    static QVector<QPair<QPoint, QColor> > findTeeth(QImage input);
    static QVector<QVariant> findPulp(QImage input, QPoint startingPoint);

    
private:
    static QImage thresholdImage(QImage input, int cutoff);
    static QVector<int> findOcculsion(QImage input);
    static QVector<QPoint> findOcculsionFaster(QImage input);
    static qreal vectorSum(QImage input, QPoint start, int angle);
    static QVector<QPoint> findOcculsionSlower(QImage input);
    static float calculateCenterValue(QImage input,int seeX,int seeY);
    static QVector<QLine> findEnamel(QImage input, QVector<QPoint> points, int cutOff);
    static QPair<QVector<QPoint>,QVector<QPoint> > findOutline(QImage input, int cutoff, QVector<QPoint> occlusion);
    static qreal findStdevArea(QImage input, QPoint center, int radius);
    static qreal findStdevArea(QImage input, QPoint center, int width, int height);
    static QVector<QPoint> findSameX(QPoint needle, QVector<QPoint> haystack);
    static QVector<QPoint> findSameY(QPoint needle, QVector<QPoint> haystack);
    static qreal calcVerticalConstrast(QImage input, QPoint center, int radius);
    static QVector<QPoint> findInterProximal(QImage input, QVector<QPoint> occPoints,QVector<QPoint> outlinePoints, int cutOff);
    static QList<QVector<QPoint> > groupPoints(QVector<QPoint> points, int width, int height, int hozDiff, int verDiff);
    static QVector<QPoint> findValidNeighbors(QPoint point, int** quickMap, int width, int height, int hozJump, int verJump);
    static QList<QVector<QPoint> > findEmbrasures(QList<QVector<QPoint> > interProxGroups,
                                                  QVector<QPoint> occu, QVector<QPoint> maxOutline, QVector<QPoint> manOutline);
    static QVector<QPoint> findInterProximalEnamel(QImage input, QList<QVector<QPoint> > interProxGroups);
    static QList<QPoint> findOddPoints(QList<QVector<QPoint> > enamelGroups, QImage input);
    static QVector<int> regionVals(int startX, int startY, int r, QImage img);
    static qreal regionAvg(int startX, int startY, int r, QImage img);
    static qreal regionAvg(int startX, int startY, int w, int h, QImage img);
    static QPoint closestPoint(QPoint start, QVector<QPoint> ends);
    static QVector<QPoint> makeLine(QPoint start, QPoint end);

};

#endif // IMAGEPROCESSOR_H
