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

        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40,65,Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(-12,63))
                                .lineToConstantHeading(new Vector2d(-12,17))
                                .turn(Math.toRadians(-55))
                                .back(3)
                                .setReversed(false)
                                .splineTo(new Vector2d(-60,13),Math.toRadians(180))
                                //.setReversed(true)
                                //.splineTo(new Vector2d(-12,17),Math.toRadians(215))

                                .build()




                );


        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)

                .addEntity(blueLeft)

                .start();
    }
}