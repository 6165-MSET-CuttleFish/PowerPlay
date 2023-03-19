package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.Robot

class BackgroundCR(val robot: Robot)
{
    var packet: TelemetryPacket= TelemetryPacket()
    val dashboard: FtcDashboard= FtcDashboard.getInstance()


    fun startHW()
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
                    var zone=Context.robot!!.pipeline.output
                    Context.signalSleeveZone=zone
                    Context.tel!!.addData("Camera 1: ", Context.signalSleeveZone)
                }
                if(Context.autoalignEnabled)
                {
                    Context.tel!!.addData("Autoalign Camera: ", Context.autoalignCameraPastInit)
                }
                if(Context.isAuto)
                {
                    Context.tel!!.addData("Signal Sleeve Camera: ", Context.signalsleeveCameraPastInit);
                }
                if(Context.signalsleeveCameraPastInit)
                {
                    Context.tel!!.addData("Signal Sleeve Max Pixels Read: ", Context.robot!!.pipeline.max2);
                }
                if(Context.autoalignCameraPastInit/*&&!Context.dashboardCameraStreaming*/)
                if(Context.autoalignCameraPastInit&&!Context.dashboardCameraStreaming)
                {
                    dashboard.startCameraStream(robot.turretCamera, 10.0);
                    Context.dashboardCameraStreaming=true
                }
                Context.tel!!.update()
                robot.slides.update()
                robot.turret.update()
            }

            //throughout the opmode
            while(!robot.l.isStopRequested)
            {
                robot.slides.update()
                robot.turret.update()
                //Context.tel!!.update()
            }
            Context.resetValues()
        }
    }
}
