package org.firstinspires.ftc.teamcode.detection.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Testing extends OpenCvPipeline
{
    Mat cropped=new Mat();
    Mat LAB=new Mat();
    Mat L=new Mat();
    Mat reconvertedRGB=new Mat();
    Mat blurred=new Mat();
    Mat filtered=new Mat();
    Rect rectCrop=new Rect();

    @Override
    public Mat processFrame(Mat input) {

        rectCrop=new Rect(0, 0, input.width(), input.height()/2);
        cropped=input.submat(rectCrop);
        //Mat blur=new Mat();
        //Imgproc.blur(input, blur, new Size(2,2));

        Imgproc.cvtColor(cropped, LAB, Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(LAB, L, 0);

        CLAHE cl=Imgproc.createCLAHE(2, new Size(3, 3));
        cl.apply(L, L);
        Core.insertChannel(L, LAB, 0);

        Imgproc.cvtColor(LAB, reconvertedRGB, Imgproc.COLOR_Lab2RGB);
        Imgproc.GaussianBlur(reconvertedRGB, blurred, new Size(3, 3), 0);
        Imgproc.bilateralFilter(blurred, filtered, 15, 75, 75);

        return  filtered;
    }
}
