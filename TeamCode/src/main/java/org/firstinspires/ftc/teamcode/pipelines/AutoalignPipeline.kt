package org.firstinspires.ftc.teamcode.pipelines

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.modules.vision.AutoalignProcessor
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline


class AutoalignPipeline: OpenCvPipeline()
{
    private val detector: AutoalignProcessor = AutoalignProcessor();
    private var currentMat: Mat?=Mat();

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