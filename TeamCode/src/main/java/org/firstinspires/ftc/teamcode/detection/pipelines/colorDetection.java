package org.firstinspires.ftc.teamcode.detection.pipelines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Camera;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//old one that might be used
public class colorDetection extends OpenCvPipeline
{

    Mat LAB=new Mat();
    Mat L=new Mat();
    Mat reconvertedRGB=new Mat();
    Mat blurred=new Mat();
    Mat filtered=new Mat();
    Mat croppedRect=new Mat();
    Mat returnMat=new Mat();
    Mat processedInput=new Mat();
    Mat preview=new Mat();

    Mat A=new Mat();
    Mat B=new Mat();
    Scalar a;
    Scalar b;

    Rect rect=new Rect(80, 140, 100, 100);

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
        /*Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(LAB, L, 0);

        CLAHE cl=Imgproc.createCLAHE(2, new Size(3, 3));
        cl.apply(L, L);
        Core.insertChannel(L, LAB, 0);

        Imgproc.cvtColor(LAB, reconvertedRGB, Imgproc.COLOR_Lab2RGB);
        Imgproc.GaussianBlur(reconvertedRGB, blurred, new Size(3, 3), 0);
        Imgproc.bilateralFilter(blurred, filtered, 15, 75, 75);*/
        Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);

        croppedRect= LAB.submat(rect);
        return croppedRect;
    }

    public void updateZone(Mat input)
    {
        Core.extractChannel(input, A, 1);
        Core.extractChannel(input, B, 2);

        a = Core.mean(A);
        b = Core.mean(B);

        double aAvg = a.val[0] - 40;
        double bAvg = b.val[0] - 40;

        tel.addData("A", aAvg);
        tel.addData("B", bAvg);

        //pink
        if (bAvg * 1.5 < aAvg) {
            state = 1;
        }
        //green
        else if (aAvg * 1.5 < bAvg) {
            state = 2;
        }
        //red
        else if (aAvg - bAvg <= 20 && aAvg > 100) {
            state = 3;
        }
    }
    //120

    @Override
    public Mat processFrame(Mat input)
    {
        processedInput=preProcessing(input);
        Imgproc.cvtColor(processedInput, returnMat, Imgproc.COLOR_Lab2RGB);
        updateZone(processedInput);

        tel.addData("State", state);
        tel.update();

        //preview=input.clone();
        //Imgproc.rectangle(preview, rect, new Scalar (0, 255, 0));
        return returnMat;
    }

    public int getOutput()
    {
        //temporary return
        return state;
    }
}
