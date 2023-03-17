package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Robot

object Context
{
    @JvmField
    var opMode: LinearOpMode? = null

    @JvmField
    var tel: Telemetry? = null

    @JvmField
    var robot: Robot? = null

    @JvmField
    var isAuto: Boolean = false

    @JvmField
    var hardwareMap: HardwareMap? = null

    @JvmField
    var hallEffectEnabled: Boolean=true

    @JvmField
    var contextPastInit: Boolean=false

    @JvmField
    var signalSleeveZone: Int = -1

    @JvmField
    var autoalignEnabled: Boolean=false;

    @JvmField
    var autoalignCameraPastInit: Boolean = false

    @JvmField
    var signalsleeveCameraPastInit: Boolean = false

    @JvmField
    var side: Side=Side.UNASSIGNED

    @JvmField
    var dashboardCameraStreaming: Boolean=false

    @JvmStatic fun updateValues()
    {
        if (opMode!!.javaClass.isAnnotationPresent(Autonomous::class.java))
        {
            isAuto = true
            hallEffectEnabled=false
            autoalignEnabled=true
        }

        if (opMode!!.javaClass.isAnnotationPresent(Left::class.java))
        {
            side=Side.LEFT
        }
        else if (opMode!!.javaClass.isAnnotationPresent(Right::class.java))
        {
            side=Side.RIGHT
        }

        tel= opMode!!.telemetry
        hardwareMap= opMode!!.hardwareMap
        contextPastInit=true
    }

    @JvmStatic fun resetValues()
    {
        opMode=null
        tel=null
        robot=null
        hardwareMap=null
        contextPastInit=false
    }
}