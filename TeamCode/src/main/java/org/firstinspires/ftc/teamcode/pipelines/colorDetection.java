package org.firstinspires.ftc.teamcode.pipelines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Camera;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class colorDetection extends OpenCvPipeline
{

    Rect rectCrop=new Rect(80, 140, 100, 100);
    CLAHE cl=Imgproc.createCLAHE(2, new Size(3, 3));


    Mat cropped=new Mat();
    Mat LAB=new Mat();
    Mat L=new Mat();
    Mat reconvertedRGB=new Mat();
    Mat blurred=new Mat();
    Mat filtered=new Mat();
    Mat HSV=new Mat();

    Mat H=new Mat();

    int state=-1;

    Telemetry tel;
    public colorDetection(Telemetry tel)
    {
        this.tel=tel;
        state=-1;
    }

    private void release()
    {
        cropped.release();
        LAB.release();
        L.release();
        cl.collectGarbage();
        reconvertedRGB.release();
        blurred.release();
        filtered.release();
        HSV.release();
    }

    public Mat preProcessing(Mat input)
    {
        //pre-processing stuff

        Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(LAB, L, 0);

        cl.apply(L, L);
        Core.insertChannel(L, LAB, 0);

        Imgproc.cvtColor(LAB, reconvertedRGB, Imgproc.COLOR_Lab2RGB);
        Imgproc.GaussianBlur(reconvertedRGB, blurred, new Size(3, 3), 0);
        Imgproc.bilateralFilter(blurred, filtered, 15, 75, 75);
        Imgproc.cvtColor(filtered, HSV, Imgproc.COLOR_RGB2HSV);
        cropped=HSV.submat(rectCrop);

        return cropped;
    }

    public void getZone(Mat input)
    {
        Core.extractChannel(input, H, 0);
        //Mat H;

        Scalar h=Core.mean(H);

        double hAvg=h.val[0];


        tel.addData("H", hAvg);

        //bocchi 💀
        if(hAvg>145&&hAvg<170)
        {
            state=1;
        }
        //ryo my beloved 😳
        else if(hAvg>105&&hAvg<125)
        {
            state=2;
        }
        //nijika-chwan(ty mr flamer) 👀
        else if(hAvg>16&&hAvg<35)
        {
            state=3;
        }
    }

    //120

    @Override
    public Mat processFrame(Mat input)
    {
        release();
        getZone(preProcessing(input));
        tel.addData("State", state);
        tel.update();

        Mat preview=input.clone();
        Imgproc.rectangle(preview, rectCrop, new Scalar (0, 255, 0));
        return preview;
    }

    public int getOutput()
    {
        if(state>0)
        {
            return state;
        }
        else
        {
            //play gacha lottery
            return (int) (Math.random()*3)+1;
        }
    }
}
