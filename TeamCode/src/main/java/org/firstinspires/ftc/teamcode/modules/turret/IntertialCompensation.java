package org.firstinspires.ftc.teamcode.modules.turret;

public class IntertialCompensation {
    public double lengthMultiplier(double extensionPos){
        //uses rightservo position of the horizontal linkage
        double length = extensionPos*29.6371 + 6.44;
        return length;
    }
    public double PIDMultiplier(double extensionPos){
        //p = Iw
        //I = 4/3ML^2
        //p = 4/3ML^2 * w
        //w = p/(4/3ML^2)
        //treat p/(4/3M) as a constant
        //w ~= 1/L^2
        double multiplier = 1/(lengthMultiplier(extensionPos)*lengthMultiplier(extensionPos));
        return multiplier;
    }
}
