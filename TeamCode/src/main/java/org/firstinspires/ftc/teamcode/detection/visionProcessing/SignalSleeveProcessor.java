package org.firstinspires.ftc.teamcode.detection.visionProcessing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;

public class SignalSleeveProcessor
{
    Mat LAB=new Mat();
    Mat L=new Mat();
    Mat reconvertedRGB=new Mat();
    Mat blurred=new Mat();
    Mat filtered=new Mat();
    Mat croppedRect=new Mat();

    Mat A=new Mat();
    Mat B=new Mat();
    Scalar a;
    Scalar b;

    Rect rect=new Rect(80, 140, 100, 100);

    public int getState(Mat input)
    {
        Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(LAB, L, 0);

        CLAHE cl=Imgproc.createCLAHE(2, new Size(3, 3));
        cl.apply(L, L);
        Core.insertChannel(L, LAB, 0);

        Imgproc.cvtColor(LAB, reconvertedRGB, Imgproc.COLOR_Lab2RGB);
        Imgproc.GaussianBlur(reconvertedRGB, blurred, new Size(3, 3), 0);
        Imgproc.bilateralFilter(blurred, filtered, 15, 75, 75);
        Imgproc.cvtColor(filtered, LAB, Imgproc.COLOR_RGB2Lab);

        croppedRect= LAB.submat(rect);

        Core.extractChannel(croppedRect, A, 1);
        Core.extractChannel(croppedRect, B, 2);

        a = Core.mean(A);
        b = Core.mean(B);

        double aAvg = a.val[0] - 40;
        double bAvg = b.val[0] - 40;

        //pink
        if (bAvg * 1.5 < aAvg) {
            return 1;
        }
        //green
        else if (aAvg * 1.5 < bAvg) {
            return 2;
        }
        //red
        else if (aAvg - bAvg <= 20 && aAvg > 100) {
            return 3;
        }
        return -1;
    }
}
