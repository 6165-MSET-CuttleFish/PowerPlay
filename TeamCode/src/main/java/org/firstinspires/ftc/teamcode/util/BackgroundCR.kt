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
    var counter: Int=0;
    var counter2: Int=0;

    fun startHW()
    {
        val job = GlobalScope.launch(Dispatchers.Default)
        {
            while(!l.isStopRequested)
            {
                l.telemetry.addData("Slides: ", slides.slidesLeft.power);
                l.telemetry.update();

                packet.put("Slides", slides.slidesLeft.power)
                dashboard.sendTelemetryPacket(packet)

                slides.update()
                turret.update()

                //counter++
                delay(20)
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
