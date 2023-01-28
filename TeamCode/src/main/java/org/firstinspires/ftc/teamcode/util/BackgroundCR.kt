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

class BackgroundCR(val robot: RobotTemp)
{
    var counter: Int=0;
    var counter2: Int=0;
    var packet: TelemetryPacket= TelemetryPacket()
    val dashboard: FtcDashboard= FtcDashboard.getInstance()

    fun startHW()
    {
        val job = GlobalScope.launch(Dispatchers.Main)
        {
            while(!robot.l.isStopRequested)
            {
                //l.telemetry.addData("Slides: ", slides.slidesLeft.power);
                //l.telemetry.update();

                try
                {
                    packet.put("Slides", robot.slides.slidesLeft.power)
                    dashboard.sendTelemetryPacket(packet)

                    robot.slides.update()
                    robot.turret.update()
                }
                catch(e: Exception)
                {
                    robot.l.telemetry.addData("Error: ", e);
                    robot.l.telemetry.update()
                }
                //counter++
            }
        }

        /*val job2=GlobalScope.launch(Dispatchers.Main)
        {
            while(true)
            {
                l.telemetry.addData("Turret: ", counter2);
                l.telemetry.update();

                packet.put("Turret", counter2)
                dashboard.sendTelemetryPacket(packet)

                counter++;
                delay(50)
            }
        }

        GlobalScope.launch(Dispatchers.Default)
        {
            while(!l.isStopRequested)
            {
                delay(50)
            }
            //job.cancel()
            //job2.cancel()
        }*/
    }
}
