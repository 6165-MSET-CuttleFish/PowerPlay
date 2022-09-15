package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.*;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 600 pixels
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity BlueSideRedTerm = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, 60, Math.toRadians(0)))
                                .strafeTo(new Vector2d(36, 24))

                                .strafeTo(new Vector2d(36, 12))
                                .setReversed(false)
                                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                                .setReversed(true)
                                .splineTo(new Vector2d(35, 12), Math.toRadians(180))

                                .setReversed(false)
                                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                                .setReversed(true)
                                .splineTo(new Vector2d(35, 12), Math.toRadians(180))

                                .setReversed(false)
                                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                                .setReversed(true)
                                .splineTo(new Vector2d(35, 12), Math.toRadians(180))

                                .setReversed(false)
                                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                                .setReversed(true)
                                .splineTo(new Vector2d(35, 12), Math.toRadians(180))

                                .setReversed(false)
                                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                                .setReversed(true)
                                .splineTo(new Vector2d(35, 12), Math.toRadians(180))
                                .strafeTo(new Vector2d(35,34)).setReversed(true)
                                //If left
                                //.splineTo(new Vector2d(12, 34), Math.toRadians(180))
                                //If mid

                                //If Right
                                //.setReversed(false)
                                //.splineTo(new Vector2d(60, 34), Math.toRadians(0))

                                .build()
                );
        RoadRunnerBotEntity BlueSideBlueTerm = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(180)))
                                .strafeTo(new Vector2d(-36, 24))

                                .strafeTo(new Vector2d(-36, 12))
                                .setReversed(false)
                                .splineTo(new Vector2d(-56, 12), Math.toRadians(180))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, 12), Math.toRadians(0))

                                .setReversed(false)
                                .splineTo(new Vector2d(-56, 12), Math.toRadians(180))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, 12), Math.toRadians(0))

                                .setReversed(false)
                                .splineTo(new Vector2d(-56, 12), Math.toRadians(180))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, 12), Math.toRadians(0))

                                .setReversed(false)
                                .splineTo(new Vector2d(-56, 12), Math.toRadians(180))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, 12), Math.toRadians(0))

                                .setReversed(false)
                                .splineTo(new Vector2d(-56, 12), Math.toRadians(180))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, 12), Math.toRadians(0))
                                .strafeTo(new Vector2d(-35,34))
                                //If left
                                //.setReversed(false)
                                //.splineTo(new Vector2d(-60, 34), Math.toRadians(180))
                                //If mid

                                //If Right
                                //.setReversed(true)
                                //.splineTo(new Vector2d(-12, 34), Math.toRadians(0))

                                .build()
                );
        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueSideRedTerm)
                .addEntity(BlueSideBlueTerm)
                .start();
    }
}