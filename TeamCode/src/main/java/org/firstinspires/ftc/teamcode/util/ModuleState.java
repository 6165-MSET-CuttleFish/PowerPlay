package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface ModuleState
{
    public Double getValue();
    public DcMotor.RunMode runMode();
}
