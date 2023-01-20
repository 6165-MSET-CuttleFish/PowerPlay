package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class AutoalignCopium extends OpenCvPipeline
{
    Telemetry tel;
    public AutoalignCopium(Telemetry tel)
    {
        this.tel=tel;
    }

    Scalar yellowLow=new Scalar(12, 60, 20);
    Scalar yellowHigh=new Scalar(30, 255, 255);
    Scalar redLow1=new Scalar(0, 60, 20);
    Scalar redHigh1=new Scalar(7, 255, 255);
    Scalar redLow2 =new Scalar(170, 60, 20);
    Scalar redHigh2 =new Scalar(180, 255, 255);
    Scalar blueLow=new Scalar(105, 60, 20);
    Scalar blueHigh=new Scalar(125, 255, 255);

    Rect rectCrop=new Rect();

    Mat cropped=new Mat();
    Mat LAB=new Mat();
    Mat L=new Mat();
    Mat reconvertedRGB=new Mat();
    Mat blurred=new Mat();
    Mat filtered=new Mat();
    Mat HSV=new Mat();

    Mat yellow=new Mat();
    Mat clone=new Mat();
    Mat blue=new Mat();
    Mat red1=new Mat();
    Mat red2 =new Mat();
    Mat combined=new Mat();
    Mat combined2=new Mat();
    Mat redCombined=new Mat();
    Mat morph1=new Mat();
    Mat morph2=new Mat();
    Mat finalMat=new Mat();
    Mat hierarchy=new Mat();
    Mat mask01=new Mat();
    List<MatOfPoint> Contours=new ArrayList<MatOfPoint>();
    Mat morphOpenKernel= Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    Mat morphCloseKernel=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
    CLAHE cl=Imgproc.createCLAHE(2, new Size(3, 3));

    MatOfKeyPoint blobs=new MatOfKeyPoint();
    List<KeyPoint> blobsList=new ArrayList<KeyPoint>();

    SimpleBlobDetector_Params params=new SimpleBlobDetector_Params();
    SimpleBlobDetector detector;


    private void release()
    {
        cropped.release();
        LAB.release();
        L.release();
        reconvertedRGB.release();
        blurred.release();
        filtered.release();
        HSV.release();
        yellow.release();
        blue.release();
        red1.release();
        red2.release();
        redCombined.release();
        combined.release();
        combined2.release();
        morph1.release();
        morph2.release();
        finalMat.release();
        mask01.release();
        morphOpenKernel.release();
        morphCloseKernel.release();
        hierarchy.release();
        blobs.release();
        clone.release();
        Contours.clear();
        cl.collectGarbage();
        //blobsList.clear();
    }

    @Override
    public void init(Mat input)
    {
        params.set_filterByArea(false);
        params.set_filterByConvexity(false);
        params.set_filterByCircularity(false);
        params.set_filterByInertia(false);

        params.set_minArea(0);
        params.set_minCircularity(0);
        params.set_minConvexity(0);
        params.set_minInertiaRatio(0);

        detector=SimpleBlobDetector.create(params);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        release();
        clone=input.clone();

        /*rectCrop=new Rect(0, 0, input.width(), input.height()/2);
        cropped=input.submat(rectCrop);

        //pre-processing stuff

        Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(LAB, L, 0);

        cl.apply(L, L);
        Core.insertChannel(L, LAB, 0);

        Imgproc.cvtColor(LAB, reconvertedRGB, Imgproc.COLOR_Lab2RGB);*/
        Imgproc.GaussianBlur(input, blurred, new Size(3, 3), 0);
        Imgproc.bilateralFilter(blurred, filtered, 11, 75, 75);
        //Imgproc.cvtColor(filtered, HSV, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(filtered, HSV, Imgproc.COLOR_RGB2HSV);


        //isolate colors

        Core.inRange(HSV, yellowLow, yellowHigh, yellow);
        Core.inRange(HSV, redLow1, redHigh1, red1);
        Core.inRange(HSV, redLow2, redHigh2, red2);
        Core.inRange(HSV, blueLow, blueHigh, blue);

        Core.bitwise_or(yellow, blue, combined);
        Core.bitwise_or(red1, red2, redCombined);
        Core.bitwise_or(combined, redCombined, combined2);

        Imgproc.morphologyEx(combined2, morph1, Imgproc.MORPH_OPEN, morphOpenKernel);
        Imgproc.morphologyEx(morph1, morph2, Imgproc.MORPH_CLOSE, morphCloseKernel);

        Imgproc.findContours(morph2, Contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        /*for(int i=0; i<Contours.size(); i++)
        {
            mask01=new Mat(input.rows(), input.cols(), CvType.CV_8U, Scalar.all(0));
            Imgproc.drawContours(mask01, Contours, i, new Scalar(255, 255, 255), -1);
        }*/
        int contourIndex=0;
        double contourArea=0;

        for(int i=0; i<Contours.size(); i++)
        {
            if(Imgproc.contourArea(Contours.get(i))>contourArea)
            {
                contourArea=Imgproc.contourArea(Contours.get(i));
                contourIndex=i;
            }
        }

        //mask01=new Mat(input.rows(), input.cols(), CvType.CV_8U, Scalar.all(0));
        Imgproc.drawContours(clone, Contours, contourIndex, new Scalar(0, 0, 0), -1);


        tel.addData("Why", Contours.size());
        tel.update();

        return clone;
    }
}
