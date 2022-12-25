package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting3 {
    public static void main(String[] args) {
        // Declare a MeepMeep instance

        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity rightAuto = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34.5,65,Math.toRadians(270)))
                                //read signal
                                .lineToConstantHeading(new Vector2d(-34.5,34.5))
                                .turn(Math.toRadians(45)) //switch with turret
                                .forward(7) //switch with extension
                                .back(7) //switch with extension
                                .turn(Math.toRadians(-45)) //switch with turret
                                .lineToConstantHeading(new Vector2d(-34.5,12))
                                .turn(Math.toRadians(-90))
                                .lineToConstantHeading(new Vector2d(-58,12)) //switch with extension
                                .lineToConstantHeading(new Vector2d(-34.5,12)) //switch with extension
                                .turn(Math.toRadians(-135))
                                .forward(7) //switch with extension
                                .back(7) //switch with extension
                                .turn(Math.toRadians(135))
                                .lineToConstantHeading(new Vector2d(-58,12)) //switch with extension
                                .lineToConstantHeading(new Vector2d(-34.5,12)) //switch with extension
                                .turn(Math.toRadians(-135))
                                .forward(7) //switch with extension
                                .back(7) //switch with extension
                                .turn(Math.toRadians(135))
                                .lineToConstantHeading(new Vector2d(-58,12)) //switch with extension
                                .lineToConstantHeading(new Vector2d(-34.5,12)) //switch with extension
                                .turn(Math.toRadians(-135))
                                .forward(7) //switch with extension
                                .back(7) //switch with extension
                                .turn(Math.toRadians(135))
                                .lineToConstantHeading(new Vector2d(-58,12)) //switch with extension
                                .lineToConstantHeading(new Vector2d(-34.5,12)) //switch with extension
                                .turn(Math.toRadians(-135))
                                .forward(7) //switch with extension
                                .back(7) //switch with extension
                                .turn(Math.toRadians(-45))
                                .lineToConstantHeading(new Vector2d(-12,12))


                                .build()




                );

        RoadRunnerBotEntity leftAuto = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(34.5,65,Math.toRadians(270)))
                                //read signal
                                .lineToConstantHeading(new Vector2d(34.5,34.5))
                                .turn(Math.toRadians(-45)) //switch with turret
                                .forward(7) //switch with extension
                                .back(7) //switch with extension
                                .turn(Math.toRadians(45)) //switch with turret
                                .lineToConstantHeading(new Vector2d(34.5,12))
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(58,12)) //switch with extension
                                .lineToConstantHeading(new Vector2d(34.5,12)) //switch with extension
                                .turn(Math.toRadians(135))
                                .forward(7) //switch with extension
                                .back(7) //switch with extension
                                .turn(Math.toRadians(-135))
                                .lineToConstantHeading(new Vector2d(58,12)) //switch with extension
                                .lineToConstantHeading(new Vector2d(34.5,12)) //switch with extension
                                .turn(Math.toRadians(135))
                                .forward(7) //switch with extension
                                .back(7) //switch with extension
                                .turn(Math.toRadians(-135))
                                .lineToConstantHeading(new Vector2d(58,12)) //switch with extension
                                .lineToConstantHeading(new Vector2d(34.5,12)) //switch with extension
                                .turn(Math.toRadians(135))
                                .forward(7) //switch with extension
                                .back(7) //switch with extension
                                .turn(Math.toRadians(-135))
                                .lineToConstantHeading(new Vector2d(58,12)) //switch with extension
                                .lineToConstantHeading(new Vector2d(34.5,12)) //switch with extension
                                .turn(Math.toRadians(135))
                                .forward(7) //switch with extension
                                .back(7) //switch with extension
                                .turn(Math.toRadians(45))
                                .lineToConstantHeading(new Vector2d(60,12))


                                .build()




                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)

                .addEntity(rightAuto)
                .addEntity(leftAuto)

                .start();
    }
}