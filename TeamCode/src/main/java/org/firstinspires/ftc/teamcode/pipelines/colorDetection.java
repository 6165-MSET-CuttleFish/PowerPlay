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

    Rect rectCrop=new Rect(260, 70, 50, 50);
    CLAHE cl=Imgproc.createCLAHE(2, new Size(3, 3));

    double hAvg;

    Scalar yellowLower=new Scalar(20, 60, 20);
    Scalar yellowMax=new Scalar(40, 255, 255);

    Scalar blueLower=new Scalar(95, 60, 20);
    Scalar blueHigher=new Scalar(125, 255, 255);

    Scalar pinkLower=new Scalar(0, 0, 140);
    Scalar pinkHigher=new Scalar(10, 80, 180);



    Mat cropped=new Mat();
    Mat LAB=new Mat();
    Mat L=new Mat();
    Mat reconvertedRGB=new Mat();
    Mat blurred=new Mat();
    Mat filtered=new Mat();
    Mat HSV=new Mat();
    Mat preProcessed=new Mat();
    Mat test=new Mat();

    int pinkCount;
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
        /*Since Nijika is the paragon of human virtue without equal past or present, she is
        most resplendent in love, tributes and accolades. Waking or sleeping, I must not
        forget Nijika’s great boon and in order to return her favor by day and by night,
        I should only think of fulfilling my loyalty. Who is Nijika? For the blind, she
        is their vision. For the deaf, she is their music. For the mute, she is their voice.
        For the anosmic, she is their aroma. For the numb, she is their feeling. For the atrophied,
         she is their muscle. For the starved, she is their sustenance. For the thirsty, she
         is their water. For the exhausted, she is their energy. For the depressed, she is their
          happiness. For the disillusioned, she is their hope. For the pessimistic, she is their
           optimism. For the disadvantaged, she is their champion. For the marginalized, she is
            their justice. For the oppressed, she is their salvation. For the righteous, she is
             their symbol. For the enlightened, she is their muse. For the erudite, she is their
              education. If Nijika speaks, I listen. If Nijika questions, I answer. If Nijika
               orders, I obey. If Nijika opines, I agree. If Nijika fears, I assure. If Nijika
                hopes, I dream. If Nijika is happy, I am jubilant. If Nijika is angry, I am
                apoplectic. If Nijika is sad, I am disconsolate. Nijika is my ideal, Nijika is
                 my romance, Nijika is my passion. Nijika is my strength, Nijika is my compass,
                  Nijika is my destination. Nijika is my language, Nijika is my culture, Nijika
                   is my religion. Nijika is my ocean, Nijika is my mountain, Nijika is my sky,
                    Nijika is my air, Nijika is my sun, Nijika is my moon, Nijika is my world.
                     Nijika is history, Nijika is present, Nijika is future. If Nijika has a
                      million fans, I am one of them. If Nijika has a thousand fans, I am one
                       of them. If Nijika has a hundred fans, I am one of them. If Nijika has ten
                        fans, I am one of them. If Nijika has only one fan, that is me. If Nijika
                         has no fans, I no longer exist. If the whole universe is for Nijika, then
                          I am for the whole universe. If the whole universe is against Nijika,
                           then I am against the whole universe. I will love, cherish, and protect
                            Nijika until my very last breath; my successors will love, cherish
                             and protect Nijika until their very last breath.*/

        //nijika-chwan(ty mr flamer) 👀
        else if(hAvg>35&&hAvg<65)
        {
            state=3;
        }
    }

    public void getZone2(Mat input)
    {
        pinkCount=0;
        blueCount=0;
        yellowCount=0;

        for(int r=0; r<input.width(); r++)
        {
            for(int c=0; c<input.height(); c++)
            {
                HVal=input.get(r, c)[0];
                SVal=input.get(r, c)[1];
                VVal=input.get(r, c)[2];
                if(HVal>120&&HVal<170)
                {
                    pinkCount++;
                }
                else if(HVal<9&&SVal<80&&VVal>140&&VVal<180)
                {
                    pinkCount++;
                }
                else if(HVal>20&&HVal<40&&SVal>60&&VVal>20)
                {
                    yellowCount++;
                }
                else if(HVal>95&&HVal<125&&SVal>60&&VVal>20)
                {
                    blueCount++;
                }
            }
        }

        max1=Math.max(pinkCount, yellowCount);
        max2=Math.max(max1, blueCount);

        if(max2==pinkCount)
            state=1;
        else if(max2==blueCount)
            state=2;
        else if(max2==yellowCount)
            state=3;
    }

    //120

    @Override
    public Mat processFrame(Mat input)
    {
        release();

        preProcessed=preProcessing(input);

        getZone2(preProcessed);
        tel.addData("State", state);
        tel.update();

        Mat preview=input.clone();
        Imgproc.rectangle(preview, rectCrop, new Scalar (0, 255, 0));

        Core.inRange(HSV, pinkLower, pinkHigher, test);

        return test;
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
