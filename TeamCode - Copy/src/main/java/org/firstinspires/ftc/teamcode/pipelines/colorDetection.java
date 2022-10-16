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
        Mat blur=new Mat();
        Imgproc.blur(input, blur, new Size(3,3));

        Mat colorSpace1=new Mat();
        Imgproc.cvtColor(blur, colorSpace1, Imgproc.COLOR_RGB2Lab);

        Mat colorLetter1=new Mat();
        Core.extractChannel(colorSpace1, colorLetter1, 0);

        Imgproc.equalizeHist(colorLetter1, colorLetter1);
        Core.insertChannel(colorLetter1, colorSpace1, 0);

        Mat RGB=new Mat();
        Imgproc.cvtColor(colorSpace1, RGB, Imgproc.COLOR_Lab2RGB);

        Mat colorSpace2=new Mat();
        Imgproc.cvtColor(RGB, colorSpace2, Imgproc.COLOR_RGB2YCrCb);

        List<Mat> channels=new ArrayList<Mat>();
        Core.split(colorSpace2, channels);
        CLAHE cl=Imgproc.createCLAHE(2.5, new Size(8, 8));
        for(Mat m:channels)
        {
            cl.apply(m, m);
        }
        Core.merge(channels, colorSpace2);
        Imgproc.cvtColor(colorSpace2, colorSpace2, Imgproc.COLOR_YCrCb2RGB);

        //Mat canYouNotBeNormal=new Mat();
        //Core.normalize(input, canYouNotBeNormal, 20, 200, Core.NORM_MINMAX);
        //canYouNotBeNormal=normalizeV2(laCringe);
        //LAB
        Mat LAB =new Mat();
        Imgproc.cvtColor(colorSpace2, LAB, Imgproc.COLOR_RGB2Lab);

        Mat temp2=new Mat();
        Rect rect=new Rect(50, 100, 100, 100);

        temp2=LAB.submat(rect);


        //Mat mask01=new Mat(LAB.rows(), LAB.cols(), CvType.CV_8U, Scalar.all(-1));
        //Imgproc.rectangle(mask01, rect, new Scalar(255), -1);
        //Core.bitwise_and(temp, temp, temp2, mask01);



        return temp2;
    }

    public Mat getZone(Mat input)
    {
        //Mat H;

        Mat A, B;
        A=new Mat();
        B=new Mat();
        //=new Mat();
        Core.extractChannel(input, A, 1);
        Core.extractChannel(input, B, 2);
        // Core.extractChannel(input, H, 0);

        Scalar a=Core.mean(A);
        Scalar b=Core.mean(B);

        double aAvg=a.val[0]-40;
        double bAvg=b.val[0]-40;

        // Scalar hAvg=Core.mean(H);

        tel.addData("A", aAvg);
        tel.addData("B", bAvg);

       /* if(hAvg.val[0]<40)
        {
            state=1;
        }
        else if(hAvg.val[0]>140)
        {
            state=2;
        }
        else
            {
                state=3;
            }
*/
        //pink
        if(bAvg*1.5<aAvg)
        {
            state=1;
        }
        //green
        else if(aAvg*1.5<bAvg)
        {
            state=2;
        }
        //red
        else if(aAvg-bAvg<=20&&aAvg>100)
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
        tel.addData("State", state);
        tel.update();

        return temp;
    }

    public int getOutput()
    {
        //temporary return
        return state;
    }
}
