package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.*
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.RobotTemp
import org.firstinspires.ftc.teamcode.modules.slides.Slides
import org.firstinspires.ftc.teamcode.modules.turret.Turret

class BackgroundCR(val turret: Turret, val slides: Slides, val l: LinearOpMode, val dashboard: FtcDashboard, val packet: TelemetryPacket)
{
    fun startHW()
    {
        val job = GlobalScope.launch(Dispatchers.Main)
        {
            while(isActive)
            {
                l.telemetry.addData("Slides: ", "Updating");
                l.telemetry.update();

                packet.put("Slides", "Updating")
                dashboard.sendTelemetryPacket(packet)

                slides.update()
                delay(30)
            }
        }

        val job2=GlobalScope.launch(Dispatchers.Main)
        {
            while(isActive)
            {
                l.telemetry.addData("Turret: ", "Updating");
                l.telemetry.update();

                packet.put("Turret", "Updating")
                dashboard.sendTelemetryPacket(packet)

                turret.update()
                delay(30)
            }
        }

        GlobalScope.launch(Dispatchers.Default)
        {
            while(!l.isStopRequested)
            {
                delay(5)
            }
            job.cancel()
            job2.cancel()
        }
    }
}
