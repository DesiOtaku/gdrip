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

QImage ImageProcessor::thresholdImage(QImage input, int cutoff) {

//    QVector<int> occLineVals = regValsBezier(p0x,p0y,p2x,p2y,bestBezX,bestBezY,3,img);
//    int sum=0;
//    foreach(int val, occLineVals) {
//        sum+=val;
//    }
//    float avgRegVals = ((float)sum) / occLineVals.count();
//    float devSum =0;
//    foreach(int val, occLineVals) {
//        float dev = val-avgRegVals;
//        devSum += (dev *dev);
//    }
//    float stdDev =(float)(sqrt(devSum / (occLineVals.count()-1)));

//    qDebug()<<"Avg: " <<avgRegVals;
//    qDebug()<<"StDev: " <<stdDev;

//    float thres = avgRegVals + (3 * stdDev);
    //now show that threshold
    QImage returnMe(input);
    for(int x=0;x<input.width();x++) {
        for(int y=0;y<input.height();y++) {
            int val = qRed(input.pixel(x,y));
            if(val > cutoff) {
                //returnMe.setPixel(x,y,255);
                returnMe.setPixel(x,y,qRgb(255,255,255));
            } else {
                returnMe.setPixel(x,y,0);
            }
            //returnMe.setPixel(x,y,qRgb(val,val,val));
        }
    }
    return returnMe;
}

QVector<int> ImageProcessor::findOcculsion(QImage input) {
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

    //QPainter painter(&input);
    //painter.fillRect(0,bestLeftY,square,square,QColor(255,0,0));

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
    //painter.fillRect(input.width()-1-square,bestRightY,square,square,QColor(255,0,0));

    //now for the ugly part: making the (Quadratic) bezier curve

    int p0x = 0;
    int p0y = bestLeftY;

    int p2x = input.width() -1;
    int p2y = bestRightY;

    int bestBezX=0;
    int bestBezY=0;
    int bestBezVal = INT_MAX;
    QProgressDialog process(tr("Calculating different bezier curves...."),
                            QString(),0,input.width()*input.height());
    process.setWindowTitle(tr("Calculating curves"));
    process.setWindowModality(Qt::WindowModal);
    for(int x=0;x<input.width();x++) {
        for(int y=0;y<input.height();y++) {
            int val = ImageProcessor::computeBezierSum(
                        p0x,p0y,p2x,p2y,x,y,bestBezVal,input);
            if(val < bestBezVal) {
                bestBezX = x;
                bestBezY = y;
                bestBezVal = val;
            }
            process.setValue(process.value() +1);
        }
    }
    process.setValue(input.width()*input.height());

    QVector<int> returnMe;
    returnMe.append(p0x); //0
    returnMe.append(p0y);//1
    returnMe.append(bestBezX);//2
    returnMe.append(bestBezY);//3
    returnMe.append(p2x);//4
    returnMe.append(p2y);//5

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

void ImageProcessor::drawBezier(int p0x, int p0y, int p2x,
                                int p2y, int p1x, int p1y, QPainter *img) {
    for(float t = 0;t<1;t+= 0.0001) {
        float onemt = 1-t;

        float x = (onemt*onemt*p0x) +
                (2 * onemt * t *p1x) +
                (t * t *p2x);

        float y = (onemt*onemt*p0y) +
                (2 * onemt * t *p1y) +
                (t * t *p2y);

        img->fillRect((int)x,(int)y,1,1,QColor(0,255,0));
        //img.setPixel((int)x,(int)y,qRgb(0,255,0));
    }

//    qDebug()<<"p0x: " << p0x;
//    qDebug()<<"p0y: " << p0y;
//    qDebug()<<"p2x: " << p2x;
//    qDebug()<<"p2y: " << p2y;

}

QImage ImageProcessor::drawOcculsion(QImage input) {
    QVector<int> occ = ImageProcessor::findOcculsion(input);
    QImage returnMe(input);
    returnMe = returnMe.convertToFormat(QImage::Format_RGB32);
    QPainter p(&returnMe);
    ImageProcessor::drawBezier(
                occ.at(0),
                occ.at(1),
                occ.at(4),
                occ.at(5),
                occ.at(2),
                occ.at(3),
                &p
                );
    return returnMe;
}

int ImageProcessor::computeBezierSum(int p0x, int p0y, int p2x,
                                  int p2y, int p1x, int p1y, int best, QImage img) {
    int returnMe=0;
    float div = 1.0/ img.width() * 2;
    for(float t = 0;t<1;t+= div) {
        float onemt = 1-t;

        float x = (onemt * onemt*p0x) +
                (2 * onemt * t *p1x) +
                (t * t *p2x);

        float y = (onemt * onemt*p0y) +
                (2 * onemt * t *p1y) +
                (t * t *p2y);

        int val = qRed(img.pixel((int)x,(int)y));
        //returnMe += (val*val) ;
        returnMe += (val) ;

        if(returnMe > best) {
            return returnMe;
        }
    }

    return returnMe;
}

QVector<int> ImageProcessor::regValsBezier(int p0x, int p0y, int p2x,
                                           int p2y, int p1x, int p1y, int r, QImage img) {
    QVector<int> returnMe;
    float div = 1.0/ img.width() * 2;
    for(float t = 0;t<1;t+= div) {
        float onemt = 1-t;

        float x = (onemt * onemt*p0x) +
                (2 * onemt * t *p1x) +
                (t * t *p2x);

        float y = (onemt * onemt*p0y) +
                (2 * onemt * t *p1y) +
                (t * t *p2y);

        returnMe << regionVals((int)x,(int)y,r,img);
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

QImage ImageProcessor::findBackground(QImage input) {
    QImage returnMe = equalizeHistogram(input);
    QVector<int> occ = ImageProcessor::findOcculsion(returnMe);

    //returnMe = returnMe.convertToFormat(QImage::Format_RGB32);
    //QPainter p(&returnMe);

    QVector<int> regVals = ImageProcessor::regValsBezier(
                occ.at(0),
                occ.at(1),
                occ.at(4),
                occ.at(5),
                occ.at(2),
                occ.at(3),
                3,
                returnMe
                );

    int sum=0;
    foreach(int i ,regVals) {
        sum+=i;
    }
    double average = ((double)sum) /  regVals.count();
    double variance=0;
    foreach(int i ,regVals) {
        int val = i-average;
        variance+= (val *val);
    }
    double standardDev =sqrt(variance / regVals.count());
    returnMe = ImageProcessor::thresholdImage(returnMe,average + (standardDev*3));
    returnMe = ImageProcessor::drawOcculsion(returnMe);

    return returnMe;
}

void ImageProcessor::drawBezierDer(int p0x, int p0y, int p2x,
                                     int p2y, int p1x, int p1y,
                                   int stDev, QPainter *input) {
    input->setPen(QColor(255,0,0));
    QImage *imgInput = (QImage *) input->device();
    input->setRenderHints(QPainter::Antialiasing);

    float div = 1.0/ (imgInput->width() * 2);
    for(float t = 0;t<1;t+= div) {
        float onemt = 1-t;

        float x = (onemt * onemt*p0x) +
                (2 * onemt * t *p1x) +
                (t * t *p2x);

        float y = (onemt * onemt*p0y) +
                (2 * onemt * t *p1y) +
                (t * t *p2y);

        float slopeX = (2*onemt*(p1x-p0x)) + (2*t*(p2x-p1x));
        float slopeY = (2*onemt*(p1y-p0y)) + (2*t*(p2y-p1y));

        float negInverse = -1 * slopeX / slopeY;

        //first lets move in the positive direction
        bool goOn = true; //go ooooooooooooooooooon

        float lookAtX = x;
        float lookAtY = y;

        int paintToX =x;
        int paintToY =y;

        while(imgInput->valid(lookAtX +1,lookAtY + negInverse) && goOn) {
            lookAtX++;
            lookAtY += negInverse;
            int newVal = qRed(imgInput->pixel(lookAtX,lookAtY));
            if(newVal > stDev) {
                paintToX = lookAtX;
                paintToY = lookAtY;
                goOn = false;
            }
        }

        input->drawLine(x,y,paintToX,paintToY);

        //now do it again for the negative direction
        lookAtX = x;
        lookAtY = y;

        paintToX =x;
        paintToY =y;
        goOn = true;

        while(imgInput->valid(lookAtX -1,lookAtY - negInverse) && goOn) {
            lookAtX--;
            lookAtY -= negInverse;
            int newVal = qRed(imgInput->pixel(lookAtX,lookAtY));
            if(newVal > stDev) {
                paintToX = lookAtX;
                paintToY = lookAtY;
                goOn = false;
            }
        }

        input->drawLine(x,y,paintToX,paintToY);

    }
}

QImage ImageProcessor::findTeeth(QImage input) {
    QImage returnMe = equalizeHistogram(input);

//    QVector<int> occ = ImageProcessor::findOcculsion(returnMe);
//    returnMe = returnMe.convertToFormat(QImage::Format_RGB32);
//    QPainter p(&returnMe);
//    /*ImageProcessor::drawBezier(
//                occ.at(0),
//                occ.at(1),
//                occ.at(4),
//                occ.at(5),
//                occ.at(2),
//                occ.at(3),
//                &p
//                );*/

//    QVector<int> regVals = ImageProcessor::regValsBezier(
//                occ.at(0),
//                occ.at(1),
//                occ.at(4),
//                occ.at(5),
//                occ.at(2),
//                occ.at(3),
//                3,
//                returnMe
//                );

//    int sum=0;
//    foreach(int i ,regVals) {
//        sum+=i;
//    }
//    double average = ((double)sum) /  regVals.count();
//    double variance=0;
//    foreach(int i ,regVals) {
//        int val = i-average;
//        variance+= (val *val);
//    }
//    double standardDev =sqrt(variance / regVals.count());


//    ImageProcessor::drawBezierDer(
//                occ.at(0),
//                occ.at(1),
//                occ.at(4),
//                occ.at(5),
//                occ.at(2),
//                occ.at(3),
//                (int)(average + (2*standardDev)),
//                &p
//                );

    QVector<QPoint> points = ImageProcessor::findOcculsionFaster(returnMe);
    returnMe = returnMe.convertToFormat(QImage::Format_RGB32);
    QPainter p(&returnMe);

    int sum=0;
    foreach(QPoint point, points) {
        sum += qRed(returnMe.pixel(point));
    }
    int average = sum / points.count();
    int variance =0;
    foreach(QPoint point, points) {
        int addMeSquare = qRed(returnMe.pixel(point))  - average;
        variance += (addMeSquare*addMeSquare);
    }
    double standardDev =sqrt(variance / points.count());


    QVector<QLine> lines = ImageProcessor::findEnamel(input,points, average + (5 * standardDev));
    QVector<QLine> inter = ImageProcessor::findInterProximal(input,points, average + (5 * standardDev));


    p.setPen(QPen(QColor(255,0,0,100)));
    foreach(QLine line, lines) {
        p.drawLine(line);
    }

    p.setPen(QPen(QColor(0,0,255)));
    foreach(QLine line, inter) {
        p.drawLine(line);
    }

    foreach(QPoint point, points) {
        p.fillRect(QRect(point,QSize(1,1)),QColor(0,255,0));
    }

    return returnMe;
}

QVector<QLine> ImageProcessor::findEnamel(QImage input, QVector<QPoint> points, int cutOff) {
    QVector<QLine> returnMe;
    foreach(QPoint point, points) {
        //first go up
        bool moveOn = true;
        for(int y=point.y();(y<input.height()) && moveOn;y++) {
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

QVector<QLine> ImageProcessor::findInterProximal(QImage input, QVector<QPoint> points, int cutOff) {
    QVector<QLine> returnMe;

    /*int strikes =(int) ((.01) * input.height());
    int minWidth = (int) ((.01) * input.width());

    foreach(QPoint point, points) {
        //first go up
        int postBlackWhiteCount=0;
        int postWhiteBlackCount=0;
        bool moveOn = true;
        for(int y=point.y();(y<input.height()) && moveOn;y++) {
            if(qRed(input.pixel(point.x(),y)) > cutOff) { //white
                if(foundBlack) {
                    moveOn = false;
                    returnMe.append(QLine(point,QPoint(point.x(),y)));
                } else {
                    foundWhite = true;
                }
            } else { //black

            }
        }

        //now move down
        moveOn = true;
        for(int y=point.y();(y>0) && moveOn;y--) {
            if(qRed(input.pixel(point.x(),y)) > cutOff) {
                if(foundFirst) {
                    moveOn = false;
                    returnMe.append(QLine(point,QPoint(point.x(),y)));
                } else {
                    foundFirst = true;
                }
            }
        }
    }*/

    return returnMe;
}

QImage ImageProcessor::brightenImage(QImage original, int amount) {
    QImage returnMe(original.width(),original.height(),QImage::Format_ARGB32);
    QPainter painter(&returnMe);

    QImage addMe(original.width(),original.height(),QImage::Format_ARGB32);

    if(amount > 50) { //brighten for rizzle
        painter.setCompositionMode(QPainter::CompositionMode_Plus);
        int alpha = (int) (( (amount-50) / 50.0) * 255.0);
        addMe.fill(QColor(255,255,255,alpha));
    } else if(amount < 50) {
        painter.setCompositionMode(QPainter::CompositionMode_Multiply);
        int alpha = (int) (( (50-amount) / 50.0) * 255.0);
        addMe.fill(QColor(0,0,0,alpha));
    } else { //no point in wasting my time
        return original;
    }

    painter.drawImage(0,0,original);
    painter.drawImage(0,0,addMe);

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

QImage ImageProcessor::invertImage(QImage input) {
    QImage returnMe(input.width(),input.height(),QImage::Format_ARGB32);
    QPainter painter(&returnMe);

    for(int x=0;x<input.width();x++) {
        for(int y=0;y<input.height();y++) {
            int value = qRed(input.pixel(x,y));
            int newValue = 255 - value;
            painter.fillRect(x,y,1,1,QColor(newValue,newValue,newValue));
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
            painter.fillRect(x,y,1,1,QColor(newValue,newValue,newValue));
        }
    }

    return returnMe;
}

QImage ImageProcessor::mirrorVertically(QImage input) {
    QImage returnMe(input.width(),input.height(),QImage::Format_ARGB32);
    QPainter painter(&returnMe);
    QPixmap newInput;
    newInput.convertFromImage(input);


    //TODO: Use drawPixmapFragments
    for(int y=0;y<input.height();y++) {
        painter.drawPixmap(0,input.height()-y-1, newInput,
                           0,y, input.width(),1);
    }

    return returnMe;
}

QImage ImageProcessor::mirrorHorizontally(QImage input) {
    QImage returnMe(input.width(),input.height(),QImage::Format_ARGB32);
    QPainter painter(&returnMe);
    QPixmap newInput;
    newInput.convertFromImage(input);

    //TODO: Use drawPixmapFragments
    for(int x=0;x<input.width();x++) {
        painter.drawPixmap(input.width()-x-1,0, newInput,
                           x,0, 1,input.height() );
    }

    return returnMe;
}
