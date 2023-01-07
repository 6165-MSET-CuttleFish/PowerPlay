package org.firstinspires.ftc.teamcode.Detection.pipelines;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class Autoalign extends OpenCvPipeline
{
    @Override
    public Mat processFrame(Mat input)
    {
        //preprocessing(increase contrast, blur, etc.)
        //threshold for colors that are wanted(red, blue, yellow)
        //process that to make objects group together
        //process for blobs
        //use blob characteristics to identify correct poles & identify which pole aligning to
        //Determine distance from said blob/pole
        return null;
    }
}
