package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueRight {
    //
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-30, 62, 0))
                                .splineTo(new Vector2d(-12,58), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(-12,24))
                                .waitSeconds(1) //extake
                                .lineToConstantHeading(new Vector2d(-13, 12))
                                .lineToConstantHeading(new Vector2d(-58, 12))
                                .waitSeconds(0.5) //intake
                                .lineToConstantHeading(new Vector2d(0, 12))
                                .waitSeconds(1) //extake
                                .lineToConstantHeading(new Vector2d(-58, 12))
                                .waitSeconds(0.5) //intake
                                .lineToConstantHeading(new Vector2d(0, 12))
                                .waitSeconds(1) //extake
                                .lineToConstantHeading(new Vector2d(-58, 12))
                                .waitSeconds(0.5) //intake
                                .lineToConstantHeading(new Vector2d(0, 12))
                                .waitSeconds(1) //extake
                                .lineToConstantHeading(new Vector2d(-58, 12))
                                .waitSeconds(0.5) //intake
                                .lineToConstantHeading(new Vector2d(0, 12))
                                .waitSeconds(1) //extake
                                .lineToConstantHeading(new Vector2d(-58, 12))

                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
