package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class colorDetection extends OpenCvPipeline
{
    Mat finalMat;
    int state=1;

    Telemetry tel;
    public colorDetection(Telemetry tel)
    {
        this.tel=tel;
        state=1;
    }


    public Mat preProcessing(Mat input)
    {
        Mat canYouNotBeNormal=new Mat();
        Core.normalize(input, canYouNotBeNormal, 20, 200, Core.NORM_MINMAX);
        //canYouNotBeNormal=normalizeV2(laCringe);

        //LAB
        Mat LAB =new Mat();
        Imgproc.cvtColor(canYouNotBeNormal, LAB, Imgproc.COLOR_RGB2Lab);

        //Blur
        Mat blurred=new Mat();
        Imgproc.blur(LAB, blurred, new Size(3, 3));

        //Dilate
        Mat dilate=new Mat();
        Mat kernel=new Mat();
        kernel=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.dilate(blurred, dilate, kernel, new Point(1.5, 1.5), 1);

        Scalar lower=new Scalar(50, 1, 1);
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
        Core.bitwise_and(LAB, LAB, temp, morphed01);


        Mat temp2=new Mat();
        Rect rect=new Rect(50, 100, 50, 100);
        Mat mask01=new Mat(LAB.rows(), LAB.cols(), CvType.CV_8U, Scalar.all(0));
        Imgproc.rectangle(mask01, rect, new Scalar(255), -1);
        Core.bitwise_and(temp, temp, temp2, mask01);

        return temp2;
    }

    public Mat getZone(Mat input)
    {
        Mat A, B;
        A=new Mat();
        B=new Mat();
        Core.extractChannel(input, A, 1);
        Core.extractChannel(input, B, 2);

        Scalar aAvg=Core.mean(A);
        Scalar bAvg=Core.mean(B);

        if(bAvg.val[0]*1.7<aAvg.val[0])
        {
            state=1;
        }
        else if(aAvg.val[0]*1.7<bAvg.val[0])
        {
            state=2;
        }
        else if(aAvg.val[0]-bAvg.val[0]<=30)
            {
                state=3;
            }
        //update the state
        return input;
    }


    @Override
    public Mat processFrame(Mat input)
    {
        Mat temp=new Mat();
        temp=preProcessing(input);
        Mat temp2=new Mat();
        temp2=getZone(temp);


        return null;
    }

    public int getOutput()
    {
        //temporary return
        return state;
    }
}
