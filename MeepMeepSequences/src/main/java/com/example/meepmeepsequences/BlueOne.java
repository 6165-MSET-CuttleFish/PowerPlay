package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueOne {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-61, 35.5, 0))
                                .splineToLinearHeading(new Pose2d(-34, 35.5), 0)
                                .strafeTo(new Vector2d(-34, 0))
                                .addDisplacementMarker(() -> {
                                    //bot.score();
                                })
                                .waitSeconds(1.75)
                                .strafeTo(new Vector2d(-34, 12))
                                .turn(Math.toRadians(180))
                                .splineTo(new Vector2d(-55, 12), Math.toRadians(180))
                                .waitSeconds(1)
                                //repeated cycling
                                .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(180))
                                .waitSeconds(1.75)
                                .addDisplacementMarker(() -> {
                                    //bot.score();
                                })
                                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(180))
                                .waitSeconds(1.75)
                                .addDisplacementMarker(() -> {
                                    //bot.score();
                                })
                                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(180))
                                .waitSeconds(1.75)
                                .addDisplacementMarker(() -> {
                                    //bot.score();
                                })
                                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(180))
                                .waitSeconds(1.75)
                                .addDisplacementMarker(() -> {
                                    //bot.score();
                                })



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
