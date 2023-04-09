package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.hardware.I2cDevice
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.util.Context.hardwareMap

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
            while(!Context.opMode!!.isStarted&&!robot.l.isStopRequested&&isActive)
            {
                if(Context.isAuto)
                {
                    var zone=Context.robot!!.dualCameras.signalSleeve.output
                    Context.signalSleeveZone=zone
                    Context.tel!!.addData("Camera 1", Context.signalSleeveZone)
                }
                else
                {
                    Context.tel!!.addData("Waiting", "in init")
                }
                if(Context.autoalignEnabled)
                {
                    Context.tel!!.addData("Autoalign Camera", Context.autoalignCameraPastInit)
                }
                if(Context.autoalignCameraPastInit&&!Context.dashboardCameraStreaming)
                {
                    dashboard.startCameraStream(robot.dualCameras.turretCamera, 10.0);
                    Context.dashboardCameraStreaming=true
                }
                if(Context.isAuto)
                {
                    Context.tel!!.addData("Signal Sleeve Camera", Context.signalsleeveCameraPastInit);
                }
                if(Context.signalsleeveCameraPastInit)
                {
                    Context.tel!!.addData("Signal Sleeve Max Pixels Read", Context.robot!!.dualCameras.signalSleeve.max2);
                }
                if(Context.autoalignCameraPastInit)
                {
                    Context.tel!!.addData("Center X", Context.robot!!.dualCameras.aligner.centerX)
                    Context.tel!!.addData("Largest area", Context.robot!!.dualCameras.aligner.largestArea)
                }
                Context.robot!!.localizer.update()
                Context.tel!!.addData("Front Left Dist", Context.robot!!.localizer.frontDistLRaw)
                Context.tel!!.addData("Front Left Raw", Context.robot!!.localizer.frontDistL)
                //Context.tel!!.addData("Front Right Dist", Context.robot!!.localizer.frontDistR)
                //Context.tel!!.addData("Left Dist", Context.robot!!.localizer.leftDist)
                Context.tel!!.addData("Right Dist", Context.robot!!.localizer.rightDist)

                //Context.tel!!.addData("Front Left I2c", Context.robot!!.localizer.distfl.connectionInfo);
                //Context.tel!!.addData("Front Left I2c client", Context.robot!!.localizer.distfl.deviceClient);

                //hardwareMap!!.getAll(I2cDeviceSynch::class.java)

                //for(device: I2cDeviceSynchImplOnSimple in robot.hardwareMap!!.getAll(I2cDeviceSynchImplOnSimple::class.java))
                //{
                  //  Context.tel!!.addData(device.deviceName, device.connectionInfo)
                //}
                //Context.tel!!.addData("GI Sensor", Context.robot!!.groundIntake.distSens.connectionInfo);
                //Context.tel!!.addData("GI Sensor", Context.robot!!.groundIntake.distSens.i2cAddress);


                //if(Context.autoalignCameraPastInit/*&&!Context.dashboardCameraStreaming*/)
                Context.tel!!.update()
                robot.slides.update()
                robot.turret.update()
            }

            //throughout the opmode
            while(!robot.l.isStopRequested&&isActive)
            {
                robot.slides.update()
                robot.turret.update()
                Context.robot!!.localizer.update()
                Context.robot!!.localizer.relocalize()

                //Context.robot!!.localizer.update()
                //Context.robot!!.localizer.relocalize()
                Context.tel!!.addData("Times relocalized", Context.robot!!.localizer.localizedCount)
                Context.tel!!.addData("Turret State", Context.robot!!.turret.state)
                Context.tel!!.addData("Slides State", Context.robot!!.slides.state)
                Context.tel!!.addData("Power", Context.robot!!.dualCameras.aligner.power)
                Context.tel!!.addData("Robot Pose", Context.robot!!.poseEstimate)
                Context.tel!!.addData("Front Distance", Context.robot!!.localizer.frontDistL)

                if(Context.isAuto)
                {
                    Context.tel!!.addData("Trajectory Duration", Context.robot!!.trajectorySequenceRunner.currentSequenceDuration())
                }

                Context.tel!!.update()
                //delay(5)
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
