package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MSrightside {
    public static void main(String[] args) {
        // Declare a MeepMeep instance

        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(550);

        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, 2.3, 3, 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38,61,Math.toRadians(270)))

                                .lineToConstantHeading(new Vector2d(-36.5, 9))
                                .waitSeconds(1.5)
                                .lineToLinearHeading(new Pose2d(-36.5,9.5, Math.toRadians(180)))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-55.25, 10))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-36.5, 9))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-55.25, 10))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-36.5, 9))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-55.25, 10))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-36.5, 9))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-55.25, 10))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-36.5, 9))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-55.25, 10))
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(-36.5, 9))
                                .waitSeconds(1.5)
                                .build()





                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .start();
    }
}