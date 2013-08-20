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

#include <QProgressDialog>
#include <QDebug>
#include <QTime>

#include "math.h"

#include "imageprocessor.h"

ImageProcessor::ImageProcessor(QObject *parent) :
    QObject(parent)
{
}

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
    qreal sum=0;
    int counter=0;
    for(int x= startX - r; x < (startX + r); x++) {
        for(int y = startY-r; y< (startY+r); y++) {
            if(img.valid(x,y)) {
                sum +=qRed(img.pixel(x,y));
                counter++;
            }
        }
    }
    return sum/counter;
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
    QImage useMe = constrastImage(input,65);
    QVector<QPoint> points = ImageProcessor::findOcculsionFaster(useMe);

    int sum=0;
    foreach(QPoint point, points) {
        sum += qRed(useMe.pixel(point));
    }
    int average = sum / points.count();
    int variance =0;
    foreach(QPoint point, points) {
        int addMeSquare = qRed(useMe.pixel(point))  - average;
        variance += (addMeSquare*addMeSquare);
    }
    double standardDev =sqrt(variance / points.count());


    //QVector<QLine> lines = ImageProcessor::findEnamel(useMe,points, average + (5 * standardDev));
    qDebug()<<"Average: "<< average;
    qDebug()<<"StDev: "<< standardDev;

    QVector<QPair<QPoint, QColor> > returnMe;

    int cutoff = average + (5 * standardDev);
    QPair<QVector<QPoint>,QVector<QPoint> > outlines = ImageProcessor::findOutline(input,cutoff,points);
    QVector<QPoint> allOutlines = outlines.first + outlines.second;
    QVector<QPoint> inter = ImageProcessor::findInterProximal(input,points,allOutlines,cutoff);
    QList<QVector<QPoint> > interProxGroups = groupPoints(inter,input.width(),input.height(),1,1);
    QVector<QPoint> proximalEnamel = findInterProximalEnamel(useMe, interProxGroups);
    QList<QVector<QPoint> > enamelGroups = groupPoints(proximalEnamel,input.width(),input.height(),3,3);
    //QList<QVector<QPoint> > embrasures = findEmbrasures(interProxGroups,points,outlines.first,outlines.second);
    QList<QPoint> oddPoints = findOddPoints(enamelGroups,input);



//    foreach(QPoint point, points) {
//        QPair<QPoint, QColor> addMe(point,QColor(255,0,0,150));
//        returnMe.append(addMe);
//    }

//    foreach(QPoint point, outlines.first) { //maxillary
//        QPair<QPoint, QColor> addMe(point,QColor(0,255,0,150));
//        returnMe.append(addMe);
//    }

//    foreach(QPoint point, outlines.second) { //manibular
//        QPair<QPoint, QColor> addMe(point,QColor(0,0,255,150));
//        returnMe.append(addMe);
//    }

//    int counter=255;
//    foreach(QVector<QPoint> group, interProxGroups) {
//        QColor addColor(counter,counter/2,counter/3,150);
//        counter-= 25;
//        foreach(QPoint point, group) {
//            QPair<QPoint, QColor> addMe(point,addColor);
//            returnMe.append(addMe);
//        }
//    }

//    foreach(QPoint point, proximalEnamel) {
//        QPair<QPoint, QColor> addMe(point,QColor(200,5,5,150));
//        returnMe.append(addMe);
//    }

//    int counter=0;
//    foreach(QVector<QPoint> group, enamelGroups) {
//        QColor addColor(QColor::colorNames().at(counter++));
//        addColor.setAlpha(150);
//        foreach(QPoint point, group) {
//            QPair<QPoint, QColor> addMe(point,addColor);
//            returnMe.append(addMe);
//        }
//    }

    foreach(QPoint point, oddPoints) {
        QPair<QPoint, QColor> addMe(point,QColor(200,5,5,150));
        returnMe.append(addMe);
    }

//    counter=255;
//    foreach(QVector<QPoint> group, embrasures) {
//        QColor addColor(counter/3,counter,counter,150);
//        counter-= 25;
//        foreach(QPoint point, group) {
//            QPair<QPoint, QColor> addMe(point,addColor);
//            returnMe.append(addMe);
//        }
//    }

//    foreach(QVector<QPoint> group, interProxGroups) {
//        qDebug()<<group.count();
//    }

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

QImage ImageProcessor::invertImage(QImage input) {
    QImage returnMe(input);
    returnMe.invertPixels();
    return returnMe;
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

QVector<QLine> ImageProcessor::findEnamel(QImage input, QVector<QPoint> points, int cutOff) {
    QVector<QLine> returnMe;
    foreach(QPoint point, points) {
        //first go up
        bool moveOn = true;
        for(int y=point.y();(y<input.height()) && moveOn;y++) {
            //qDebug()<<qRed(input.pixel(point.x(),y));
            if(qRed(input.pixel(point.x(),y)) > cutOff) {
                moveOn = false;
                returnMe.append(QLine(point,QPoint(point.x(),y)));
            }
        }

        //now move down
        moveOn = true;
        for(int y=point.y();(y>0) && moveOn;y--) {
            if(qRed(input.pixel(point.x(),y)) > cutOff) {
                moveOn = false;
                returnMe.append(QLine(point,QPoint(point.x(),y)));
            }
        }
    }

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
    QImage constrastedImg = constrastImage(input,70);
    for(int lookAtX=0;lookAtX<input.width();lookAtX++) {
        for(int lookAtY=0;lookAtY<input.height();lookAtY++) {
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

QList<QVector<QPoint> > ImageProcessor::groupPoints(QVector<QPoint> points, int width, int height, int hozDiff=1, int verDiff=1) {
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
    QProgressDialog dia(tr("Doing Connectivity checks"),QString(),0,width*height);
    dia.setWindowModality(Qt::WindowModal);
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

            dia.setValue(dia.value() +1);
        }
    }
    dia.setValue(width*height);

    QList<QVector<QPoint> > realReturnMe;

    foreach(QVector<QPoint> group, returnMe) {
        if(group.count() > 750) { //more than 750 continous points
            qDebug()<<group.count();
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



QImage ImageProcessor::constrastImage(QImage original, int amount) {
    QImage returnMe(original.width(),original.height(),QImage::Format_ARGB32);
    QPainter painter(&returnMe);


    //Taken from Wikipedia / GIMP at http://en.wikipedia.org/wiki/Image_editing#Contrast_change_and_brightening
    double frac = ((amount - 50)*2) / 100.0;
    for(int x=0;x<original.width();x++) {
        for(int y=0;y<original.height();y++) {
            double value = (qRed(original.pixel(x,y)))/255.0;
            double newVal = (value - 0.5) * (tan ((frac + 1) * 0.78539816339) ) + 0.5;
            newVal = qMin(newVal*255,255.0);
            newVal = qMax(newVal,0.0);
            int setVal = (int)newVal;
            painter.fillRect(x,y,1,1,QColor(setVal,setVal,setVal));
        }
    }


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



QVector<QPoint> ImageProcessor::findInterProximalEnamel(QImage input,
                                                      QList<QVector<QPoint> > interProxGroups) {
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




QVector<QVariant> ImageProcessor::findPulp(QImage input, QPoint startingPoint) {
    QVector<QVariant> returnMe;
    int endRad = 100;
    int segmentSize =5;
    int segLeft = segmentSize/-2;
    qreal pi = 3.14159265;


    for(qreal rads=0;rads < (2*pi);rads += .01) {
        int stageStatus =0; //each stage is changed in each major pixel value change
        //qreal radians = (deg* 3.14159265)/180;
        qreal radians = rads;
        QVector<qreal> segmentVariances;
        qreal maxVariance=0;
        qreal maxVarianceR =0;
        for(int r=1;r<endRad;r++) {
            //qreal xPoint = startingPoint.x() + (r * cos(radians));
            //qreal yPoint = startingPoint.y() + (r * sin(radians));
            //QPoint currentPoint((int)xPoint,(int)yPoint); //TODO: bilinear interpolation
            //now figure out the segment variance value
            //QVector<int> values;
            qreal weighSum =0;
            for(int m=r;m<=r+segmentSize;m++) {
                qreal xCheck = startingPoint.x() + (m * cos(radians));
                qreal yCheck = startingPoint.y() + (m * sin(radians));
                QPoint lookAt((int)xCheck,(int)yCheck);
                if(input.valid(lookAt)) {
                    int val = qRed(input.pixel(lookAt));
                    weighSum += pow(segLeft + (m-r),3)*val;
                } else {
                    m=r+segmentSize;
                    weighSum =0;
                }
            }
            weighSum = qAbs(weighSum);
            if(weighSum > maxVariance) {
                maxVariance = weighSum;
                maxVarianceR = r;
            }

//            if(maxVariance > 10) {
//                qDebug()<<"Winner was: " << maxVariance;
//                //int m = r + (segmentSize/2);
//                int m = maxVarianceR + (segmentSize/2);
//                qreal xCheck = startingPoint.x() + (m * cos(radians));
//                qreal yCheck = startingPoint.y() + (m * sin(radians));
//                returnMe.append(QPoint((int)xCheck,(int)yCheck));
//                //r = endRad;
//            }
        }
        qDebug()<<"Winner was: " << maxVariance;
        if(maxVariance > 4000) {
            qDebug()<<"Winner was: " << maxVariance;
            //int m = r + (segmentSize/2);
            int m = maxVarianceR + (segmentSize/2);
            qreal xCheck = startingPoint.x() + (m * cos(radians));
            qreal yCheck = startingPoint.y() + (m * sin(radians));
            returnMe.append(QPoint((int)xCheck,(int)yCheck));
            //r = endRad;
        }

//        for(int m=maxVarianceR;m<=(maxVarianceR+segmentSize);m++) {
//            qreal xCheck = startingPoint.x() + (m * cos(radians));
//            qreal yCheck = startingPoint.y() + (m * sin(radians));
//            returnMe.append(QPoint((int)xCheck,(int)yCheck));
//        }
    }

    return returnMe;

}
