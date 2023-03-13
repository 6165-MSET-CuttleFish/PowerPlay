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
            while(!Context.contextPastInit&&!Context.opMode!!.isStopRequested)
            {

            }

            //during init
            while(!Context.opMode!!.isStarted&&!Context.opMode!!.isStopRequested)
            {
                if(Context.isAuto)
                {
                    var zone=Context.robot!!.pipeline.output
                    if(zone>0)
                    {
                        Context.signalSleeveZone=zone
                    }
                    Context.tel!!.addData("Camera 1: ", Context.signalSleeveZone)
                }
                Context.tel!!.update()
                robot.slides.update()
                robot.turret.update()
            }

            //right at opmode start
            if(Context.autoalignEnabled)
            {
                robot.initAutoAlignCamera()
            }

            //throughout the opmode
            while(!robot.l.isStopRequested)
            {
                robot.slides.update()
                robot.turret.update()
                Context.tel!!.update()

                if(Context.autoalignCameraPastInit)
                {
                    dashboard.startCameraStream(robot.turretCamera, 10.0);
                    Context.autoalignCameraPastInit=false;
                }
            }
        }
    }
}
