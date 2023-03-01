package org.firstinspires.ftc.teamcode.pipelines;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.util.Left;
import org.firstinspires.ftc.teamcode.util.Right;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class colorDetection extends OpenCvPipeline
{

    Rect rectCropLeft=new Rect(146, 20, 60, 60);
    Rect rectCropRight=new Rect(146, 20, 60, 60);
    Rect rectCrop;

    CLAHE cl=Imgproc.createCLAHE(2, new Size(3, 3));

    double hAvg;

    Scalar yellowLower=new Scalar(20, 60, 20);
    Scalar yellowMax=new Scalar(40, 255, 255);

    Scalar blueLower=new Scalar(95, 60, 20);
    Scalar blueHigher=new Scalar(125, 255, 255);

    Scalar greenLower =new Scalar(55, 60, 20);
    Scalar greenHigher =new Scalar(80, 255, 255);



    Mat cropped=new Mat();
    Mat LAB=new Mat();
    Mat L=new Mat();
    Mat reconvertedRGB=new Mat();
    Mat blurred=new Mat();
    Mat filtered=new Mat();
    Mat HSV=new Mat();
    Mat preProcessed=new Mat();
    Mat test=new Mat();

    int greenCount;
    int yellowCount;
    int blueCount;

    int max1;
    int max2;

    double HVal;
    double SVal;
    double VVal;

    Mat H=new Mat();

    int state=-1;

    Telemetry tel;
    public colorDetection(LinearOpMode l)
    {
        if(l.getClass().isAnnotationPresent(Left.class))
        {
            rectCrop=rectCropLeft;
        }
        else if(l.getClass().isAnnotationPresent(Right.class))
        {
            rectCrop=rectCropRight;
        }

        this.tel=l.telemetry;
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
        preProcessed.release();
        test.release();
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

        hAvg=h.val[0];


        tel.addData("H", hAvg);

        //bocchi 💀  green
        if(hAvg>145&&hAvg<170)
        {
            state=1;
        }
        //ryo my beloved 😳 blue
        else if(hAvg>105&&hAvg<125)
        {
            state=2;
        }

        //nijika-chwan(ty mr flamer) 👀 yellow
        else if(hAvg>35&&hAvg<65)
        {
            state=3;
        }
    }

    public void getZone2(Mat input)
    {
        greenCount =0;
        blueCount=0;
        yellowCount=0;

        if(input!=null)
        {
            for(int r=0; r<input.width(); r++)
            {
                for(int c=0; c<input.height(); c++)
                {
                    try
                    {
                        HVal=input.get(r, c)[0];
                        SVal=input.get(r, c)[1];
                        VVal=input.get(r, c)[2];

                        if(HVal>55&&HVal<80)
                        {
                            greenCount++;
                        }
                            /*else if(HVal<10&&SVal<80&&VVal>140&&VVal<180)
                            {
                                pinkCount++;
                            }*/
                        else if(HVal>20&&HVal<40&&SVal>60&&VVal>20)
                        {
                            yellowCount++;
                        }
                        else if(HVal>95&&HVal<125&&SVal>60&&VVal>20)
                        {
                            blueCount++;
                        }
                    }
                    catch(NullPointerException e)
                    {
                        //cancer
                        state--;
                        throw e;
                    }
                }
            }

            max1=Math.max(greenCount, yellowCount);
            max2=Math.max(max1, blueCount);

            if(max2==0)
                state=-1;
            else if(max2== greenCount)
                state=1;
            else if(max2==blueCount)
                state=2;
            else if(max2==yellowCount)
                state=3;
        }


    }

    //120

    @Override
    public Mat processFrame(Mat input)
    {
        release();

        preProcessed=preProcessing(input);

        getZone2(preProcessed);
        //tel.addData("State", state);
        //tel.update();

        Mat preview=input.clone();
        Imgproc.rectangle(preview, rectCropLeft, new Scalar (0, 255, 0));

        Core.inRange(HSV, greenLower, greenHigher, test);

        return preview;
    }

    public int getOutput()
    {
        return state;
    }
    public double getHAvg()
    {
        return hAvg;
    }

    public int getMax()
    {
        return max2;
    }

}