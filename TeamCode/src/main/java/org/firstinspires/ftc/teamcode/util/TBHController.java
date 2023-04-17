package org.firstinspires.ftc.teamcode.util;

public class TBHController
{
    double gain;
    double targetVel;
    double currentVel;
    double power;
    double prevError;
    double TBH;
    boolean firstCross;


    public TBHController(double gain)
    {
        this.gain=gain;
    }

   public void setTargetVel(double targetVel)
   {
       if(this.targetVel==targetVel)
       {
           return;
       }
       else
       {
           this.targetVel=targetVel;
           firstCross=true;
           TBH=0;
       }

       if(targetVel==0)
       {
           power=0;
       }
   }
   public double update(double currentVel)
   {
       this.currentVel=currentVel;

       double error=targetVel-currentVel;

       power+=gain*error;


        if(firstCross)
        {
            firstCross=false;
            if(targetVel!=0)
            {
                power=Math.signum(targetVel)*0.25;
            }
            else
            {
                power=0;
            }
            TBH=power;
        }
       if(Math.signum(prevError)!=Math.signum(error))
       {
           power=0.5*(power+TBH);
           TBH=power;
       }

        prevError=error;

       return power;
   }
}
