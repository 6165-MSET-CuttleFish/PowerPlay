package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class yes {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //i l joos 16236
        //cuttlefish auto???
        //thanks sid and hunter
        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep).setDimensions(16,17)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())

                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(-360), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(35, 62, Math.toRadians(90)))
                                        .lineToConstantHeading(new Vector2d(35,15))
                                        .lineToLinearHeading(new Pose2d(31,7,Math.toRadians(45)))
                                        .waitSeconds(1)
                                        .splineTo(new Vector2d(60, 12),Math.toRadians(0))
                                        .waitSeconds(1)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(31, 7),Math.toRadians(225))
                                        .waitSeconds(1)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(60, 12),Math.toRadians(0))
                                        .waitSeconds(1)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(31, 7),Math.toRadians(225))
                                        .waitSeconds(1)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(60, 12),Math.toRadians(0))
                                        .waitSeconds(1)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(31, 7),Math.toRadians(225))
                                        .waitSeconds(1)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(60, 12),Math.toRadians(0))
                                        .waitSeconds(1)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(31, 7),Math.toRadians(225))
                                        .waitSeconds(1)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(60, 12),Math.toRadians(0))
                                        .waitSeconds(1)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(31, 7),Math.toRadians(225))
                                        .waitSeconds(1)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(60, 12),Math.toRadians(0))
                                        .build()
                );
        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep).setDimensions(16,17)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())

                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(-360), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(-90)))
                                .lineToConstantHeading(new Vector2d(-35,15))
                                .lineToLinearHeading(new Pose2d(-31,7,Math.toRadians(135)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-60, 12),Math.toRadians(180))
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(-31, 7),Math.toRadians(315))
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineTo(new Vector2d(-60, 12),Math.toRadians(180))
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(-31, 7),Math.toRadians(315))
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineTo(new Vector2d(-60, 12),Math.toRadians(180))
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(-31, 7),Math.toRadians(315))
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineTo(new Vector2d(-60, 12),Math.toRadians(180))
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(-31, 7),Math.toRadians(315))
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineTo(new Vector2d(-60, 12),Math.toRadians(180))
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(-31, 7),Math.toRadians(315))
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineTo(new Vector2d(-60, 12),Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .addEntity(bot2)
         //  .addEntity(mySecondBot)
        .start();
    }
}