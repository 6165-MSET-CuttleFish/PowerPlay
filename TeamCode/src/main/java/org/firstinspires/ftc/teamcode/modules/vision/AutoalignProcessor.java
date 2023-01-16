package org.firstinspires.ftc.teamcode.modules.vision;

import com.google.ftcresearch.tfod.tracking.ObjectTracker;

import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class AutoalignProcessor
{
    Scalar yellowLow=new Scalar(0, 0, 0);
    Scalar yellowHigh=new Scalar(0, 0, 0);
    Scalar redLow=new Scalar(0, 0, 0);
    Scalar redHigh=new Scalar(0, 0, 0);
    Scalar blueLow=new Scalar(0, 0, 0);
    Scalar blueHigh=new Scalar(0, 0, 0);

    Rect rectCrop=new Rect();

    Mat cropped=new Mat();
    Mat LAB=new Mat();
    Mat L=new Mat();
    Mat reconvertedRGB=new Mat();
    Mat blurred=new Mat();
    Mat filtered=new Mat();
    Mat HSV=new Mat();

    Mat yellow=new Mat();
    Mat blue=new Mat();
    Mat red=new Mat();
    Mat combined=new Mat();
    Mat combined2=new Mat();
    Mat morph1=new Mat();
    Mat morph2=new Mat();
    Mat morphOpenKernel=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    Mat morphCloseKernel=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));

    MatOfKeyPoint blobs=new MatOfKeyPoint();
    List<KeyPoint> blobsList;

    double xDist;
    double ticksApproximation;
    double aFactor;

    SimpleBlobDetector_Params params=new SimpleBlobDetector_Params();
    SimpleBlobDetector detector;

    public AutoalignProcessor()
    {
        params.set_filterByArea(true);
        params.set_filterByConvexity(true);
        params.set_filterByCircularity(true);
        params.set_filterByInertia(true);

        params.set_minArea(0);
        params.set_minCircularity(0);
        params.set_minConvexity(0);
        params.set_minInertiaRatio(0);

        detector=SimpleBlobDetector.create(params);
    }

    public double getTicks(Mat currentMat)
    {
        //take top half because that's all that is relevant

        rectCrop=new Rect(0, 0, currentMat.width(), currentMat.height()/2);
        cropped=currentMat.submat(rectCrop);

        //pre-processing stuff

        Imgproc.cvtColor(cropped, LAB, Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(LAB, L, 0);

        CLAHE cl=Imgproc.createCLAHE(2, new Size(3, 3));
        cl.apply(L, L);
        Core.insertChannel(L, LAB, 0);

        Imgproc.cvtColor(LAB, reconvertedRGB, Imgproc.COLOR_Lab2RGB);
        Imgproc.GaussianBlur(reconvertedRGB, blurred, new Size(3, 3), 0);
        Imgproc.bilateralFilter(blurred, filtered, 15, 75, 75);
        Imgproc.cvtColor(filtered, HSV, Imgproc.COLOR_RGB2HSV);


        //isolate colors

        Core.inRange(HSV, yellowLow, yellowHigh, yellow);
        Core.inRange(HSV, redLow, redHigh, red);
        Core.inRange(HSV, blueLow, blueHigh, blue);

        Core.bitwise_or(yellow, red, combined);
        Core.bitwise_or(combined, blue, combined2);

        Imgproc.morphologyEx(combined2, morph1, Imgproc.MORPH_OPEN, morphOpenKernel);
        Imgproc.morphologyEx(morph1, morph2, Imgproc.MORPH_CLOSE, morphCloseKernel);

        //detect blobby blobs :). Alternatively can use contours here to figure out same thing.

        detector.detect(morph2, blobs);
        blobsList=blobs.toList();

        //do some processing to determine which blob we care about

        int index=-1;
        for(int i=0; i<blobsList.size(); i++)
        {
            index=i;
        }

        //use selected blob and find distance in order to figure out how much to turn

        xDist=blobsList.get(index).pt.x;
        ticksApproximation=xDist*aFactor;

        return ticksApproximation;
    }
}