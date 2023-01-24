package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import kotlinx.coroutines.*
import org.firstinspires.ftc.robotcore.external.Telemetry

class TestCoroutine(val telemetry: Telemetry, val dashboard: FtcDashboard, val packet: TelemetryPacket)
{
    var test: Int=0;

    fun startCoroutine()
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            while(isActive)
            {
                telemetry.addData("Thing: ", "Running");
                telemetry.update()

                packet.put("Help", test)
                dashboard.sendTelemetryPacket(packet)
                test++;

                delay(50)
            }
        }
    }
}