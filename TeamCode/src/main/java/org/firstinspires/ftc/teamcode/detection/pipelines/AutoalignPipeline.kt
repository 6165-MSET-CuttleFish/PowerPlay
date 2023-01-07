package org.firstinspires.ftc.teamcode.detection.pipelines

import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.detection.visionProcessing.AutoAlignProcessor
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline

class AutoalignPipeline: OpenCvPipeline()
{
    private val detector: AutoAlignProcessor = AutoAlignProcessor();
    private var currentMat:Mat?=Mat();

    override fun processFrame(input: Mat?): Mat?
    {
        currentMat=input;
        return input;
    }

    fun getDistance(): Double
    {
        val distance=runBlocking(Dispatchers.Default)
        {
            detector.getTicks(currentMat);
        }
        return distance;
    }
}