package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.Robot

class BackgroundTasks(val robot: Robot)
{
    var packet: TelemetryPacket= TelemetryPacket()
    val dashboard: FtcDashboard= FtcDashboard.getInstance()


    fun start()
    {
        GlobalScope.launch(Dispatchers.Main)
        {

            while(!Context.contextPastInit&&!robot.l.isStopRequested)
            {

            }

            //during init
            while(!Context.opMode!!.isStarted&&!robot.l.isStopRequested)
            {
                if(Context.isAuto)
                {
                    var zone=Context.robot!!.dualCameras.signalSleeve.output
                    Context.signalSleeveZone=zone
                    Context.tel!!.addData("Camera 1: ", Context.signalSleeveZone)
                }
                else
                {
                    Context.tel!!.addData("Waiting", "in init")
                }
                if(Context.autoalignEnabled)
                {
                    Context.tel!!.addData("Autoalign Camera: ", Context.autoalignCameraPastInit)
                }
                if(Context.autoalignCameraPastInit&&!Context.dashboardCameraStreaming)
                {
                    dashboard.startCameraStream(robot.dualCameras.turretCamera, 10.0);
                    Context.dashboardCameraStreaming=true
                }
                if(Context.isAuto)
                {
                    Context.tel!!.addData("Signal Sleeve Camera: ", Context.signalsleeveCameraPastInit);
                }
                if(Context.signalsleeveCameraPastInit)
                {
                    Context.tel!!.addData("Signal Sleeve Max Pixels Read: ", Context.robot!!.dualCameras.signalSleeve.max2);
                }
                if(Context.autoalignCameraPastInit)
                {
                    Context.tel!!.addData("Center X", Context.robot!!.dualCameras.aligner.centerX)
                }
                //Context.robot!!.localizer.update()
                //Context.tel!!.addData("Front Left Dist", Context.robot!!.localizer.frontDistLRaw)
                //Context.tel!!.addData("Front Right Dist", Context.robot!!.localizer.frontDistR)
                //Context.tel!!.addData("Left Dist", Context.robot!!.localizer.leftDist)
                //Context.tel!!.addData("Right Dist", Context.robot!!.localizer.rightDist)

                //if(Context.autoalignCameraPastInit/*&&!Context.dashboardCameraStreaming*/)
                Context.tel!!.update()
                robot.slides.update()
                robot.turret.update()
            }

            //throughout the opmode
            while(!robot.l.isStopRequested)
            {
                robot.slides.update()
                robot.turret.update()
                //robot.current.update()
                if(Context.isAuto)
                {
                    Context.tel!!.addData("Turret State", Context.robot!!.turret.getState())
                    Context.tel!!.addData("Turret Autoalign Power", Context.robot!!.turret.autoalign.getPower())
                    //Context.tel!!.addData("Voltage", Context.robot!!.turret.voltSensor.voltage)
                    Context.tel!!.update()
                }
                //Context.robot!!.localizer.update()
                //Context.robot!!.localizer.relocalize()
                //Context.robot!!.localizer.update()
                //Context.tel!!.addData("Front Left Dist", Context.robot!!.localizer.frontDistL)
                //Context.tel!!.addData("Front Right Dist", Context.robot!!.localizer.frontDistR)
                //Context.tel!!.addData("Left Dist", Context.robot!!.localizer.leftDist)
                //Context.tel!!.addData("Right Dist", Context.robot!!.localizer.rightDist)
                //Context.tel!!.update()
            }
            Context.resetValues()
            //robot.dualCameras.closeCameras()
        }
    }
}
