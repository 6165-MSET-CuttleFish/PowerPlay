package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
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

    @JvmStatic fun updateValues()
    {
        if (opMode!!.javaClass.isAnnotationPresent(Autonomous::class.java))
        {
            isAuto = true
            hallEffectEnabled=false
            autoalignEnabled=true
        }

        tel= opMode!!.telemetry
        hardwareMap= opMode!!.hardwareMap
        contextPastInit=true
    }
}