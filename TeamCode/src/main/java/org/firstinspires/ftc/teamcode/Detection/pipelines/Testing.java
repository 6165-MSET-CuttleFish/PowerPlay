package org.firstinspires.ftc.teamcode.Detection.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Testing extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {

        Mat blur=new Mat();
        Imgproc.blur(input, blur, new Size(2,2));

        Mat colorSpace1=new Mat();
        Imgproc.cvtColor(blur, colorSpace1, Imgproc.COLOR_RGB2Lab);

        Mat colorLetter1=new Mat();
        Core.extractChannel(colorSpace1, colorLetter1, 0);

        CLAHE cl=Imgproc.createCLAHE(2, new Size(6, 9));

        //Imgproc.equalizeHist(colorLetter1, colorLetter1);
        cl.apply(colorLetter1, colorLetter1);


        Core.insertChannel(colorLetter1, colorSpace1, 0);

        Mat RGB=new Mat();
        Imgproc.cvtColor(colorSpace1, RGB, Imgproc.COLOR_Lab2RGB);

        return  RGB;
    }
}
