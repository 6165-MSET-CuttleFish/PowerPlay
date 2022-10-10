package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class colorDetection extends OpenCvPipeline
{
    Mat finalMat;

    Telemetry tel;
    public colorDetection(Telemetry tel)
    {
        this.tel=tel;
    }


    public Mat filterColor(Mat input)
    {
        Mat canYouNotBeNormal=new Mat();
        Core.normalize(input, canYouNotBeNormal, 20, 200, Core.NORM_MINMAX);
        //canYouNotBeNormal=normalizeV2(laCringe);

        //HSV
        Mat HSV=new Mat();
        Imgproc.cvtColor(canYouNotBeNormal, HSV, Imgproc.COLOR_RGB2HSV);

        //Blur
        Mat blurred=new Mat();
        Imgproc.blur(HSV, blurred, new Size(3, 3));

        //Dilate
        Mat dilate=new Mat();
        Mat kernel=new Mat();
        kernel=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.dilate(blurred, dilate, kernel, new Point(1.5, 1.5), 1);

        Scalar lower=new Scalar(0, 60, 40);
        Scalar higher=new Scalar(255, 255, 255);

        Mat mask=new Mat();
        Core.inRange(dilate, lower, higher, mask);

        Mat kernel1=new Mat();
        Mat morphed=new Mat();
        kernel1=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6,6));
        Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel1);

        Mat morphed01=new Mat();
        kernel1=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4,4));
        Imgproc.morphologyEx(morphed, morphed01, Imgproc.MORPH_CLOSE, kernel1);

        Mat temp=new Mat();
        Core.bitwise_and(canYouNotBeNormal, canYouNotBeNormal, temp, morphed01);

        

        return temp;
    }


    @Override
    public Mat processFrame(Mat input)
    {
        finalMat=filterColor(input);
        return null;
    }

    public Camera.State getOutput()
    {
        //temporary return
        return Camera.State.ONE;
    }
}
