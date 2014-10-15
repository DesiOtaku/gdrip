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

#include <QDebug>
#include <QTime>
#include <qmath.h>

#include "math.h"

#include "imageprocessor.h"

ImageProcessor::ImageProcessor(QObject *parent) :
    QObject(parent)
{
}


//---------------Sane functions to use----------------------------------

QImage ImageProcessor::equalizeHistogram(QImage input) {
    QImage returnMe(input.width(),input.height(),QImage::Format_ARGB32);
    QPainter painter(&returnMe);

    QVector<float> occ = ImageProcessor::findOccurrences(input);
    QVector<float> pdf;
    float currentpdf =0;
    for(int i=0;i<occ.count();i++) {
        currentpdf += occ.value(i);
        pdf.append(currentpdf);
    }

    for(int x=0;x<input.width();x++) {
        for(int y=0;y<input.height();y++) {
            int originalVal = qRed(input.pixel(x,y));
            float pdfVal = pdf.value(originalVal);
            int newVal = (int)(pdfVal * 255);
            painter.fillRect(x,y,1,1,QColor(newVal,newVal,newVal));
        }
    }
    return returnMe;
}

QVector<float> ImageProcessor::findOccurrences(QImage input, int offset) {
    QVector<float> returnMe;
    returnMe.fill(0,256);
    float single = 1.0 / (input.width() * input.height());

    for(int x=0;x<input.width();x++) {
        for(int y=0;y<input.height();y++) {
            int pixelIndex = qRed(input.pixel(x,y)) + offset;
            pixelIndex = qMax(pixelIndex,0);
            pixelIndex = qMin(pixelIndex,255);
            float newVal = returnMe.value(pixelIndex)+single;
            returnMe.replace(pixelIndex,newVal);
        }
    }

    return returnMe;
}

QImage ImageProcessor::invertImage(QImage input) {
    QImage returnMe(input);
    returnMe.invertPixels();
    return returnMe;
}


QImage ImageProcessor::spreadHistogram(QImage input) {
    //Really, this is normalization, http://en.wikipedia.org/wiki/Normalization_(image_processing)
    QImage returnMe(input.width(),input.height(),QImage::Format_ARGB32);
    QPainter painter(&returnMe);
    QVector<float> occ = ImageProcessor::findOccurrences(input);
    int lowest =0;
    bool goOn = true;

    for(int i=0;(i<occ.size()) && goOn;i++) {
        if(occ.at(i) == 0.0) {
            lowest = i;
        } else {
            goOn = false;
        }
    }

    int highest = 255;
    goOn =true;

    for(int i=occ.size()-1;(i>0) && goOn;i--) {
        if(occ.at(i) == 0.0) {
            highest = i;
        } else {
            goOn = false;
        }
    }

    int diff = highest - lowest;

    for(int x=0;x<input.width();x++) {
        for(int y=0;y<input.height();y++) {
            int value = qRed(input.pixel(x,y));
            int newValue =(int) ((value - lowest) * (255.0 / diff));
            //TODO: faster way to fill those pixels
            painter.fillRect(x,y,1,1,QColor(newValue,newValue,newValue));
        }
    }

    return returnMe;
}

QImage ImageProcessor::constrastImage(QImage original, int amount) {
    QImage returnMe(original.width(),original.height(),QImage::Format_RGB32);
    //QPainter painter(&returnMe);


    //Taken from Wikipedia / GIMP at http://en.wikipedia.org/wiki/Image_editing#Contrast_change_and_brightening
    double frac = ((amount - 50)*2) / 100.0;

    #pragma omp parallel for
    for(int x=0;x<original.width();x++) {
        for(int y=0;y<original.height();y++) {
            qreal value = (qRed(original.pixel(x,y)))/255.0;
            qreal newVal = (value - 0.5) * (tan ((frac + 1) * 0.78539816339) ) + 0.5;
            newVal = qMin(newVal*255,255.0);
            newVal = qMax(newVal,0.0);
            int setVal = (int)newVal;

            //qDebug()<<"("<<x<<","<<y<<") is now "<<setVal;
            //painter.fillRect(x,y,1,1,QColor(setVal,setVal,setVal));
            returnMe.setPixel(x,y,qRgb(setVal,setVal,setVal));
        }
    }

    return returnMe;
}

QVector<QPoint> ImageProcessor::intersection(QVector<QPoint> a, QVector<QPoint> b) {
    QVector<QPoint> returnMe;
    foreach(QPoint check, a) {
        if(b.contains(check)) {
            returnMe.append(check);
        }
    }
    return returnMe;
}

//---------------Experimental functions start here----------------------------------

QVector<QPoint> ImageProcessor::findOcculsionFaster(QImage input) {
    int radius = (int) (.1 * input.height());
    //qDebug()<<"radius: " <<radius;

    //left side
    int bestLeftY=0;
    int bestLeftYval=INT_MAX;

    for(int currentY=radius;currentY<input.height()-radius;currentY++) {
        int sum =0;
        foreach(int x,ImageProcessor::regionVals(0,currentY,radius,input)) {
            sum+=x;
        }

        if(sum < bestLeftYval) {
            bestLeftY = currentY;
            bestLeftYval = sum;
        }
    }

    //right side
    int bestRightY=0;
    int bestRightYval=INT_MAX;

    for(int currentY=radius;currentY<input.height()-radius;currentY++) {
        int sum =0;
        foreach(int x,ImageProcessor::regionVals(input.width()-1,currentY,radius,input)) {
            sum+=x;
        }
        //qDebug()<<"Sum was: " <<sum;

        if(sum < bestRightYval) {
            bestRightY = currentY;
            bestRightYval = sum;
        }
    }

    int yPen = bestLeftY;

    QVector<QPoint> returnMe;


    for(int x=0;x<input.width();x++) { //go from left to right
        int diff = bestRightY - yPen;
        if(qAbs(diff) == (input.width() - x)) {
            int movementDir = qAbs(diff)/diff; //+1 or -1 to push it in the right direction
            yPen += movementDir;
        } else {
            float penUpTotal= ImageProcessor::calculateCenterValue(input,x,yPen+1);
            float penRightTotal=ImageProcessor::calculateCenterValue(input,x,yPen);
            float penDownTotal=ImageProcessor::calculateCenterValue(input,x,yPen-1);
            float lowest = qMin(qMin(penUpTotal,penRightTotal),penDownTotal);

            if(penUpTotal == lowest) {
                yPen++;
            } else if(penDownTotal == lowest) {
                yPen--;
            }
        }
        returnMe.append(QPoint(x,yPen));
    }
    return returnMe;

}

qreal ImageProcessor::vectorSum(QImage input, QPoint start, int fakeangle) {
    bool goOn = true;
    QPoint currentPoint;
    qreal returnMe =0.0;
    int length=1;
    qreal angle = (fakeangle/180.0) * M_PI;
    while(goOn) {
        int newX = (int)(start.x() + (length * qCos(angle) ));
        int newY = (int)(start.y() + (length * qSin(angle) ));
        currentPoint = QPoint(newX,newY);
        if(input.valid(currentPoint)) {
            //qreal value =  regionAvg(currentPoint.x(),currentPoint.y(),1,1,input);
            qreal value =  qRed(input.pixel(currentPoint));
            //qDebug()<<"Adding " <<(value/length);
            returnMe += (value/sqrt(length));
            //qDebug()<<"Return me is now " <<returnMe;
            length++;
        } else {
            goOn = false;
        }
    }

    return returnMe;
}

QVector<QPoint> ImageProcessor::findOcculsionSlower(QImage input) {
    QVector<QPoint> returnMe;

    int radius = (int) (.1 * input.height());
    //qDebug()<<"radius: " <<radius;

    //left side
    int bestLeftY=0;
    int bestLeftYval=INT_MAX;
    for(int currentY=radius;currentY<input.height()-radius;currentY++) {
        int sum =0;
        foreach(int x,ImageProcessor::regionVals(0,currentY,radius,input)) {
            sum+=x;
        }

        if(sum < bestLeftYval) {
            bestLeftY = currentY;
            bestLeftYval = sum;
        }
    }

    QPoint currentPoint(0,bestLeftY);
    returnMe.append(currentPoint);
    for(int lookX =0;lookX<input.width();lookX++) {
        qreal bestAngle=0;
        qreal bestAngleValue=1024;
        for(int angle=-89;angle<90;angle++) {
            qreal angleValue = vectorSum(input,currentPoint,angle);
            if(angleValue < bestAngleValue) {
                bestAngle = angle;
                bestAngleValue = angleValue;
            }
        }
        qDebug()<<"Best angle was: " << bestAngle << " with "<< bestAngleValue;
        if(bestAngle > 15) {
            QPoint nextPoint(currentPoint.x() +1, currentPoint.y() +1);
            currentPoint = nextPoint;
        } else if(bestAngle < -15) {
            QPoint nextPoint(currentPoint.x() +1, currentPoint.y() - 1);
            currentPoint = nextPoint;
        } else {
            QPoint nextPoint(currentPoint.x() +1, currentPoint.y());
            currentPoint = nextPoint;
        }
        returnMe.append(currentPoint);
    }

    return returnMe;
}

float ImageProcessor::calculateCenterValue(QImage input, int seeX, int seeY) {
    float returnMe =0;

    for(int y=0;y<input.height();y++) {
        float distance = qMax((float)qAbs(y - seeY),(float).01);
        float value = qRed(input.pixel(seeX,y));
        returnMe+= value / distance;
    }

    return returnMe;
}


QVector<int> ImageProcessor::regionVals(int startX, int startY, int r, QImage img) {
    QVector<int> vals;
    for(int x= startX - r; x < (startX + r); x++) {
        for(int y = startY-r; y< (startY+r); y++) {
            if(img.valid(x,y)) {
                //int val = qRed(img.pixel(x,y));
                //vals.append(val*val*val);
                vals.append(qRed(img.pixel(x,y)));
            }
        }
    }

    return vals;
}

qreal ImageProcessor::regionAvg(int startX, int startY, int r, QImage img) {
    return regionAvg(startX,startY,r,r,img);
}

qreal ImageProcessor::regionAvg(int startX, int startY, int w, int h, QImage img) {
    qreal sum=0;
    int counter=0;
    for(int x= startX - w; x <= (startX + w); x++) {
        for(int y = startY-h; y<= (startY+h); y++) {
            if(img.valid(x,y)) {
                sum +=qRed(img.pixel(x,y));
                counter++;
            }
        }
    }
    return sum/counter;
}

QVector<QPair<QPoint, QColor> > ImageProcessor::findTeeth(QImage input) {
    QTime t;
    t.start();
    QImage useMe = constrastImage(input,65);
    QVector<QPoint> points = ImageProcessor::findOcculsionFaster(useMe);
    qDebug("1Time elapsed: %d ms", t.elapsed());

    QVector<float> occValues = findAverageAndStd(points,input);
    int average = (int) occValues.at(0);
    float standardDev = occValues.at(1);
    qDebug("2Time elapsed: %d ms", t.elapsed());

    int cutoff = average + (0 * standardDev);
    QPair<QVector<QPoint>,QVector<QPoint> > outlines = ImageProcessor::findOutline(input,cutoff,points);
    qDebug("3Time elapsed: %d ms", t.elapsed());
    QVector<QPoint> allOutlines = outlines.first + outlines.second;
    qDebug("4Time elapsed: %d ms", t.elapsed());
    QVector<QPoint> inter = ImageProcessor::findInterProximal(input,points,allOutlines,cutoff);
    qDebug("5Time elapsed: %d ms", t.elapsed());
    QList<QVector<QPoint> > interProxGroups = groupPoints(inter,input.width(),input.height(),1,1,750);
    qDebug("6Time elapsed: %d ms", t.elapsed());

    QVector<QPoint> enamelPoints = findInterProximalEnamel(input,interProxGroups,points);
    qDebug("7Time elapsed: %d ms", t.elapsed());
    QVector<QPoint> badTooth = findCloseDecay(interProxGroups,input);
    qDebug("8Time elapsed: %d ms", t.elapsed());
    QVector<QPoint> badEnamel = intersection(enamelPoints,badTooth);
    qDebug("9Time elapsed: %d ms", t.elapsed());



    //QList<QVector<QPoint> > groupedEnamel = groupPoints(badEnamel,input.width(),input.height(),2,2,10);

    QVector<QPair<QPoint, QColor> > returnMe;

    int counter=0;
    foreach(QVector<QPoint> group,  interProxGroups) {
        //QColor useColor = QColor(QColor::colorNames().at(counter));
        foreach(QPoint point, group) {
            QPair<QPoint, QColor> addMe(point,QColor(5,200,5,50));
            returnMe.append(addMe);

        }
        counter++;
    }

    foreach(QPoint point, badEnamel) {
        QPair<QPoint, QColor> addMe(point,QColor(200,5,5,150));
        returnMe.append(addMe);
    }


    return returnMe;
}

qreal ImageProcessor::findStdevArea(QImage input, QPoint center, int radius) {
    return findStdevArea(input,center,radius*2,radius*2);
}

qreal ImageProcessor::findStdevArea(QImage input, QPoint center, int width, int height) {
    QVector<int> localVals;
    int xStart = qMax(0,center.x()-width);
    int xEnd = qMin(input.width()-1,center.x()+width);

    int yStart = qMax(0,center.y()-height);
    int yEnd = qMin(input.height()-1,center.y()+height);

    for(int scanX=xStart;scanX<=xEnd;scanX++) {
        for(int scanY=yStart;scanY<=yEnd;scanY++) {
            localVals.append(qRed(input.pixel(scanX,scanY)));
        }
    }

    //Get the average
    qreal sum=0;
    foreach(int val, localVals) {
        sum+=val;
    }
    qreal average = sum / localVals.count();


    //Now to get the stDEV
    qreal variance=0;
    foreach(int val, localVals) {
        variance += pow(average-val,2);
    }
    return sqrt( variance / localVals.count());
}

qreal ImageProcessor::calcVerticalConstrast(QImage input, QPoint center, int radius) {
    qreal sum=0;
    int xStart = qMax(0,center.x()-radius);
    int xEnd = qMin(input.width()-1,center.x()+radius);
//    int xStart = qMax( center.x()-1,0);
//    int xEnd = qMin( center.x()+1,input.width()-1);

    int yStart = qMax(0,center.y()-radius);
    int yEnd = qMin(input.height()-1,center.y()+radius);

    int count = yEnd-yStart;

    for(int scanX=xStart;scanX<=xEnd;scanX++) {
        for(int scanY=yStart;scanY<=yEnd;scanY++) {
            int value = qRed(input.pixel(scanX,scanY));
            if(scanY < center.y()) { //white is good
                if(value > 40) {
                    sum++;
                }
            } else if(scanY > center.y()) { //black is good
                if(value < 40) {
                    sum++;
                }
            }
        }
    }
    return sum/count;
}

QPair<QVector<QPoint>,QVector<QPoint> > ImageProcessor::findOutline(QImage input, int cutoff, QVector<QPoint> occlusion) {
    //int radius = (int) (.0001 * input.width()*input.height());
    int radius = 25;
    //int radius = 50;
    QImage constrastedImg = constrastImage(input,55);
    int offSetAmount = 10;
    int pastOccAllowance = 5;

    //First, lets get the top left change point
    qreal highestStDev =0;
    int bestStartY=0;

    QVector<QPoint> maxPoints;
    QVector<QPoint> manPoints;

    QPoint leftOcc = occlusion.first();

    for(int currentY=radius;currentY<leftOcc.y();currentY++) {
        //Scan the area
        qreal stDev = calcVerticalConstrast(constrastedImg,QPoint(0,currentY),radius);

        if(stDev > highestStDev) {
            bestStartY = currentY;
            highestStDev = stDev;
        }
    }

    maxPoints.append(QPoint(0,bestStartY));
    int currentY = bestStartY;

    //Now, move right
    for(int currentX=1;currentX<input.width();currentX++) {
        int bestOffset =-1*offSetAmount;
        qreal bestOffsetValue=-1;
        for(int offsetY=-1*offSetAmount;offsetY<=offSetAmount;offsetY++) {
            qreal value = calcVerticalConstrast(constrastedImg,
                                                QPoint(currentX,currentY+offsetY),radius);
            if(value > bestOffsetValue) {
                bestOffset = offsetY;
                bestOffsetValue = value;
            }
        }

        if((currentY+bestOffset) > (occlusion.at(currentX).y() - pastOccAllowance)) {
            bestOffset = -1;
        }

        currentY+=bestOffset;
        maxPoints.append(QPoint(currentX,currentY));
    }


    //Now, do the same exact thing but for the bottom part
    //Remember, we want the LOWEST score since we want black on top
    qreal lowestStDev =INT_MAX;
    bestStartY=0;

    for(int currentY=leftOcc.y()+1;currentY<input.height();currentY++) {
        //Scan the area
        qreal stDev = calcVerticalConstrast(constrastedImg,QPoint(0,currentY),radius);

        if(stDev < lowestStDev) {
            bestStartY = currentY;
            lowestStDev = stDev;
        }
    }

    manPoints.append(QPoint(0,bestStartY));
    currentY = bestStartY;

    //Now, move right
    for(int currentX=1;currentX<input.width();currentX++) {
        int bestOffset =-10;
        qreal bestOffsetValue=INT_MAX;
        for(int offsetY=-10;offsetY<=10;offsetY++) {
            qreal value = calcVerticalConstrast(constrastedImg,
                                                QPoint(currentX,currentY+offsetY),radius);
            if(value < bestOffsetValue) {
                bestOffset = offsetY;
                bestOffsetValue = value;
            }
        }

        if((currentY+bestOffset) < (occlusion.at(currentX).y() + pastOccAllowance)) {
            bestOffset = 1;
        }

        currentY+=bestOffset;
        manPoints.append(QPoint(currentX,currentY));
    }

    QPair<QVector<QPoint>,QVector<QPoint> > returnMe(maxPoints,manPoints);
    return returnMe;
}

QVector<QPoint> ImageProcessor::findSameX(QPoint needle, QVector<QPoint> haystack) {
    QVector<QPoint> returnMe;
    foreach(QPoint hay, haystack) {
        if(needle.x() == hay.x()) {
            returnMe.append(hay);
        }
    }
    return returnMe;
}

QVector<QPoint> ImageProcessor::findSameY(QPoint needle, QVector<QPoint> haystack) {
    QVector<QPoint> returnMe;
    foreach(QPoint hay, haystack) {
        if(needle.y() == hay.y()) {
            returnMe.append(hay);
        }
    }
    return returnMe;
}

QVector<QPoint> ImageProcessor::findInterProximal(QImage input, QVector<QPoint> occPoints, QVector<QPoint> outlinePoints, int cutOff){
    QVector<QPoint> returnMe;
    QImage constrastedImg = constrastImage(input,60);
    int lowYCutoff = (int) (.05 * input.height());
    int highYCutoff = (int) (.9 * input.height());

    for(int lookAtX=0;lookAtX<input.width();lookAtX++) {
        for(int lookAtY=lowYCutoff;lookAtY<highYCutoff;lookAtY++) {
            int value = qRed(constrastedImg.pixel(lookAtX,lookAtY));
            if(value <= cutOff) {
                QVector<QPoint> occSame = findSameX(QPoint(lookAtX,lookAtY),outlinePoints);
                if(occSame.count()==2) {
                    qreal highLookAt = lookAtY - (.05 * input.height());
                    qreal lowLookAt = lookAtY + (.05 * input.height());
                    if( (lowLookAt < occSame.first().y()) && (lowLookAt < occSame.last().y()) ) { //maxillary interproximal
                        returnMe.append(QPoint(lookAtX,lookAtY));
                    } else if( (highLookAt > occSame.first().y()) && (highLookAt > occSame.last().y()) ) { //mandibular interproximal
                        returnMe.append(QPoint(lookAtX,lookAtY));
                    }
                }
            }
        }
    }
    return returnMe;
}

QVector<QPoint> ImageProcessor::findValidNeighbors(QPoint point, int **quickMap, int width, int height, int hozJump=1, int verJump=1) {
    QVector<QPoint> returnMe;
    int xStart = qMax(0,point.x()-hozJump);
    int xEnd = qMin(width-1,point.x()+hozJump);

    int yStart = qMax(0,point.y()-verJump);
    int yEnd = qMin(height-1,point.y()+verJump);

    for(int currentX=xStart;currentX<=xEnd;currentX++) {
        for(int currentY=yStart;currentY<=yEnd;currentY++) {
            if(quickMap[currentX][currentY] != -1) {
                returnMe.append(QPoint(currentX,currentY));
            }
        }
    }
    return returnMe;
}

QList<QVector<QPoint> > ImageProcessor::groupPoints(QVector<QPoint> points, int width, int height, int hozDiff, int verDiff, int minSize) {
    //http://en.wikipedia.org/wiki/Connected-component_labeling
    QList<QVector<QPoint> > returnMe; //same as "linked"
    QVector<QPoint> emptySet;

    int** map = new int*[width]; //same as "labels"
    int** quickMap = new int*[width]; //just to make a fast lookup for points
    for(int i=0;i<width;i++) {
        map[i] = new int[height];
        quickMap[i] = new int[height];
        for(int b=0;b<height;b++) {
            map[i][b] = -1; //"empty"
            quickMap[i][b] = -1;
        }
    }
    foreach(QPoint point,points) {
        quickMap[point.x()][point.y()] = 1;
        returnMe.append(emptySet);
    }

    int counter=0;//same as "NextLabel"

    //First pass
    for(int x=0;x<width;x++) { //yeah, its column then row, but it shouldn't matter
        for(int y=0;y<height;y++) {
            if(quickMap[x][y] != -1) { //if it is not background
                QPoint currentPoint(x,y);
                QVector<QPoint> neighbors = findValidNeighbors(currentPoint,map,width,height,hozDiff,verDiff);
                if(neighbors.count() == 0) {
                    returnMe[counter].append(currentPoint);
                    map[x][y] = counter;
                    counter++;
                } else {
                    int lowest= INT_MAX;
                    foreach(QPoint point, neighbors) {
                        if(map[point.x()][point.y()] < lowest) {
                            lowest = map[point.x()][point.y()];
                        }
                    }
                    map[x][y] = lowest;
                    foreach(QPoint point, neighbors) { //go though each neghbor and fix their values
                        int oldVal = map[point.x()][point.y()];
                        if(oldVal != lowest) { //force that point and its allies to the "low"
                            QVector<QPoint> oldVec = returnMe[oldVal];
                            foreach(QPoint changePoint, oldVec) { //make all of its allies a new value
                                map[changePoint.x()][changePoint.y()] = lowest;
                            }
                            returnMe[lowest] += oldVec;
                            map[point.x()][point.y()] = lowest;
                            returnMe[oldVal] = emptySet;
                        }
                    }
                    returnMe[lowest].append(currentPoint);
                }
            }
        }
    }

    QList<QVector<QPoint> > realReturnMe;

    foreach(QVector<QPoint> group, returnMe) {
        if(group.count() > minSize) { //more than 750 continous points
            //qDebug()<<group.count();
            realReturnMe.append(group);
        }
    }

    //Free up the memory we used for the arrays
    for(int i=0;i<width;i++) {
        delete[] map[i];
        delete[] quickMap[i];
    }
    delete[] map;
    delete[] quickMap;

    return realReturnMe;
}

QVector<QPoint> ImageProcessor::findCloseDecay(QList<QVector<QPoint> > interProxGroups, QImage input) {
    QVector<QPoint> returnMe;
    int jumpAmount =2;
    int counterLimit = input.width() / 50;
    foreach(QVector<QPoint> group, interProxGroups) {
        QVector<QPoint> leftPoints;
        QVector<QPoint> rightPoints;
        //first get the high and low bounds of the entire group
        int highestY = -1;
        int lowestY = INT_MAX;
        foreach(QPoint point, group) {
            if(point.y() > highestY) {
                highestY = point.y();
            }
            if(point.y() < lowestY) {
                lowestY = point.y();
            }
        }

        for(int y=lowestY;y<=highestY;y++) {
            QVector<QPoint> sameY = findSameY(QPoint(-1,y),group);
            QPoint highestX =QPoint(-1,y);
            QPoint lowestX = QPoint(INT_MAX,y);
            foreach(QPoint point, sameY) {
                if(point.x() > highestX.x()) {
                    highestX = point;
                }
                if(point.x() < lowestX.x()) {
                    lowestX = point;
                }
            }

            QPoint leftAdd = QPoint(lowestX.x()-jumpAmount,lowestX.y());
            if(input.valid(leftAdd)) {
                leftPoints.append(leftAdd);
            }

            QPoint rightAdd = QPoint(highestX.x()+ jumpAmount,highestX.y());
            if(input.valid(rightAdd)) {
                rightPoints.append(rightAdd);
            }

        }

        QVector<float> leftStats = findAverageAndStd(leftPoints,input);
        QVector<float> rightStats = findAverageAndStd(rightPoints,input);

        float leftCut = leftStats.at(0) ;
        float rightCut = rightStats.at(0);

        //now move away from the interproximal area
        foreach(QPoint point, leftPoints) { //move left from interproximal
            int value = qRed(input.pixel(point));
            if(value < leftCut) {
                returnMe.append(point);
                bool goOn = true;
                int stopAtMe = 2* value;
                QPoint leftwardEye = point;
                int counter=0;
                while(goOn) {
                    counter++;
                    leftwardEye = QPoint(leftwardEye.x()-1,leftwardEye.y());
                    qreal avg = regionAvg(leftwardEye.x(),leftwardEye.y(),2,input);
                    if(input.valid(leftwardEye) && (avg < stopAtMe) && (counter<counterLimit)) {
                        returnMe.append(leftwardEye);
                    } else {
                        goOn= false;
                    }
                }
            }
        }

        foreach(QPoint point, rightPoints) { //move right from interproximal
            int value = qRed(input.pixel(point));
            if(value < rightCut) {
                returnMe.append(point);
                bool goOn = true;
                int stopAtMe = 2* value;
                QPoint rightwardEye = point;
                int counter=0;
                while(goOn) {
                    counter++;
                    rightwardEye = QPoint(rightwardEye.x()+1,rightwardEye.y());
                    qreal avg = regionAvg(rightwardEye.x(),rightwardEye.y(),2,input);
                    if(input.valid(rightwardEye) && (avg < stopAtMe) && (counter<counterLimit)) {
                        returnMe.append(rightwardEye);
                    } else {
                        goOn= false;
                    }
                }
            }
        }
    }

    QVector<QPoint> realReturnMe;
    foreach(QPoint point,returnMe) {
        if(input.valid(point)) {
            realReturnMe.append(point);
        }
    }

    return realReturnMe;

}

QVector<float> ImageProcessor::findAverageAndStd(QVector<QPoint> values, QImage input) {
    QVector<float> returnMe;
    float sum=0;
    float variance=0;
    foreach(QPoint point, values) {
        sum +=qRed(input.pixel(point));
    }
    float average = sum/values.count();
    foreach(QPoint point, values) {
        variance += pow(average-qRed(input.pixel(point)),2);
    }

    returnMe.append(average);
    returnMe.append(sqrt(variance/values.count()));
    return returnMe;
}

QPoint ImageProcessor::closestPoint(QPoint start, QVector<QPoint> ends) {
    QPoint returnMe = ends.first();
    int bestDistance = INT_MAX;
    foreach(QPoint end, ends) {
        int currentDistance = (int) (pow(end.x()-start.x(),2) + pow(end.y()-start.y(),2));
        //don't bother with the sqrt since we are not returning the distance
        if(currentDistance < bestDistance) {
            bestDistance = currentDistance;
            returnMe = end;
        }
    }

    return returnMe;
}

QVector<QPoint> ImageProcessor::makeLine(QPoint start, QPoint end) {
    //http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm#Simplification
    QVector<QPoint> returnMe;
    int dx = qAbs(end.x() - start.x());
    int dy = qAbs(end.y() - start.y());
    int sx, sy;

    if(start.x() < end.x()) {
        sx = 1;
    } else {
        sx = -1;
    }

    if(start.y() < end.y()) {
        sy = 1;
    } else {
        sy = -1;
    }
    int err = dx-dy;
    bool goOn = true;

    while(goOn) {
        returnMe.append(start);
        if(end == start) {
            goOn = false;
        } else {
            int e2 = 2* err;
            if(e2 > (-1 * dy)) {
                err = err - dy;
                start.setX(start.x() + sx);
            }
            if(e2 < dx) {
                err = err + dx;
                start.setY(start.y() + sy);
            }
        }
    }


    return returnMe;
}

QList<QVector<QPoint> > ImageProcessor::findEmbrasures(QList<QVector<QPoint> > interProxGroups,
                                                       QVector<QPoint> occu, QVector<QPoint> maxOutline, QVector<QPoint> manOutline) {
    QList<QVector<QPoint> > returnMe;

    foreach(QVector<QPoint> group, interProxGroups) {
        //first figure out if it is max or man
        QPoint aPoint = group.first();
        QVector<QPoint> occXVal = findSameX(aPoint,occu);
        if(occXVal.count() > 0) {
            QVector<QPoint> addMe;
            if(aPoint.y() < occXVal.first().y()) { //maxillary
                //first find the lowest point
                QPoint lowest(-1,-1);
                foreach(QPoint point, group) {
                    if(point.y() > lowest.y()) {
                        lowest = point;
                    }
                }
                QPoint start(lowest);
                QPoint stop = closestPoint(lowest,maxOutline);
                addMe = makeLine(start,stop);
            } else { //mandibular
                //first find the highest point
                QPoint highest(INT_MAX,INT_MAX);
                foreach(QPoint point, group) {
                    if(point.y() < highest.y()) {
                        highest = point;
                    }
                }
                QPoint start(highest);
                QPoint stop = closestPoint(highest,manOutline);
                addMe = makeLine(start,stop);
            }
            returnMe.append(addMe);
        }
    }


    return returnMe;

}


QVector<QPoint> ImageProcessor::findInterProximalEnamel(QImage input, QList<QVector<QPoint> > interProxGroups, QVector<QPoint> occPoints) {
    QVector<QPoint> returnMe;
    int starter = (int)(input.width() * 0.005);
    int ender = (int)(input.width() * 0.05);
    foreach(QVector<QPoint> group, interProxGroups) {
        QList<QPoint> leftPoints;
        QList<QPoint> rightPoints;
        //first get the high and low bounds of the entire group
        int highestY = -1;
        int lowestY = INT_MAX;
        foreach(QPoint point, group) {
            if(point.y() > highestY) {
                highestY = point.y();
            }
            if(point.y() < lowestY) {
                lowestY = point.y();
            }
        }

        for(int y=lowestY;y<=highestY;y++) {
            QVector<QPoint> sameY = findSameY(QPoint(-1,y),group);
            QPoint highestX =QPoint(-1,y);
            QPoint lowestX = QPoint(INT_MAX,y);
            foreach(QPoint point, sameY) {
                if(point.x() > highestX.x()) {
                    highestX = point;
                }
                if(point.x() < lowestX.x()) {
                    lowestX = point;
                }
            }
            leftPoints.append(lowestX);
            rightPoints.append(highestX);
        }

        int jumpAmount = 4;
        //now move away from the interproximal area
        foreach(QPoint point, leftPoints) { //move left from interproximal
            qreal currentAverage=0;
            qreal nextAverage=0;
            QPoint currentPoint(point.x() - starter, point.y());
            QPoint nextPoint(currentPoint.x() - jumpAmount, point.y() );
            int endX = point.x() - ender;
            while ((currentAverage < (nextAverage +5) ) && (currentPoint.x() > endX)) {
                currentAverage = regionAvg(currentPoint.x(),currentPoint.y(),jumpAmount,2,input);
                nextAverage = regionAvg(nextPoint.x(),nextPoint.y(),jumpAmount,2,input);
                currentPoint = nextPoint;
                nextPoint= QPoint(currentPoint.x() - jumpAmount, point.y() );
            }
            if(input.valid(currentPoint) && (currentPoint.x() > endX)) {
                returnMe.append(currentPoint);
                while(currentPoint.x() != point.x()) {
                    currentPoint.setX(currentPoint.x()+1);
                    returnMe.append(currentPoint);
                }
            }
        }

        foreach(QPoint point, rightPoints) { //move right from interproximal
            qreal currentAverage=0;
            qreal nextAverage=0;
            QPoint currentPoint(point.x() + starter, point.y());
            QPoint nextPoint(currentPoint.x() + jumpAmount, point.y() );
            int endX = point.x() + ender;
            while ((currentAverage < (nextAverage +5) ) && (currentPoint.x() < endX)) {
                currentAverage = regionAvg(currentPoint.x(),currentPoint.y(),jumpAmount,2,input);
                nextAverage = regionAvg(nextPoint.x(),nextPoint.y(),jumpAmount,2,input);
                currentPoint = nextPoint;
                nextPoint= QPoint(currentPoint.x() + jumpAmount, point.y() );
            }
            if(input.valid(currentPoint) && (currentPoint.x() < endX)) {
                returnMe.append(currentPoint);
                while(currentPoint.x() != point.x()) {
                    currentPoint.setX(currentPoint.x()-1);
                    returnMe.append(currentPoint);
                }
            }
        }

    }
    return returnMe;
}

QList<QPoint> ImageProcessor::findOddPoints(QList<QVector<QPoint> > enamelGroups, QImage input) {
    QList<QPoint> returnMe;
    foreach(QVector<QPoint> group, enamelGroups) {
        //returnMe.append(group.toList());

        foreach(QPoint point, group) {
            qreal avg = regionAvg(point.x(),point.y(),5,input);
            //qreal stDev = findStdevArea(input,point,5,5);
            int value = qRed(input.pixel(point));
            //int cutOff = (int) qMax(0.0,avg - (stDev * 1));
            if(value < avg) {
                returnMe.append(point);
            }
        }

//        int sum=0;
//        foreach(QPoint point, group) {
//            sum += qRed(input.pixel(point));
//        }
//        qreal average = ((qreal)sum) / group.count();

//        qreal variance =0;
//        foreach(QPoint point, group) {
//            int diff = qRed(input.pixel(point)) - average;
//            variance += pow(diff,2);
//        }
//        qreal deviation = sqrt( variance / group.count());
//        int cutOff = (int) qMax(0.0,average - (deviation * 1));
//        foreach(QPoint point, group) {
//            if(qRed(input.pixel(point)) < average) {
//                returnMe.append(point);
//            }
//        }
    }

    return returnMe;
}


