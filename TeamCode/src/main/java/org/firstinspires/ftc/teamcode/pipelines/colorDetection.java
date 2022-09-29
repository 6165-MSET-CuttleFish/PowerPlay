package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.opencv.core.Mat;
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


        return new Mat();
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
