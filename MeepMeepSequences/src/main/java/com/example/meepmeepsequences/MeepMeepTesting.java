package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.*;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 600 pixels
        MeepMeep meepMeep = new MeepMeep(500);

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
                                //If Right
                                //.splineTo(new Vector2d(12, 34), Math.toRadians(180))
                                //If mid

                                //If Left
                                .setReversed(false)
                                .splineTo(new Vector2d(60, 34), Math.toRadians(0))

                                .build()
                );
        RoadRunnerBotEntity BlueSideBlueTerm = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeBlueDark())
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
                                //If Right
                                //.setReversed(false)
                                //.splineTo(new Vector2d(-60, 34), Math.toRadians(180))
                                //If mid

                                //If Left
                                .setReversed(true)
                                .splineTo(new Vector2d(-12, 34), Math.toRadians(0))

                                .build()
                );
        RoadRunnerBotEntity ComplexBlueBlue = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))

                                .splineTo(new Vector2d(-36, 24),Math.toRadians(-90))
                                .splineTo(new Vector2d(-24, 12),Math.toRadians(0))
                                .setReversed(true)
                                .splineTo(new Vector2d(-56, 12), Math.toRadians(180))
                                .setReversed(false)
                                .splineTo(new Vector2d(-35, 12), Math.toRadians(0))

                                .setReversed(true)
                                .splineTo(new Vector2d(-56, 12), Math.toRadians(180))
                                .setReversed(false)
                                .splineTo(new Vector2d(-35, 12), Math.toRadians(0))

                                .setReversed(true)
                                .splineTo(new Vector2d(-56, 12), Math.toRadians(180))
                                .setReversed(false)
                                .splineTo(new Vector2d(-35, 12), Math.toRadians(0))

                                .setReversed(true)
                                .splineTo(new Vector2d(-56, 12), Math.toRadians(180))
                                .setReversed(false)
                                .splineTo(new Vector2d(-35, 12), Math.toRadians(0))

                                .setReversed(true)
                                .splineTo(new Vector2d(-56, 12), Math.toRadians(180))
                                .setReversed(false)
                                .splineTo(new Vector2d(-35, 12), Math.toRadians(0))
                                .strafeTo(new Vector2d(-35,34))
                                //If Right
                                //.setReversed(true)
                                //.splineTo(new Vector2d(-60, 34), Math.toRadians(180))
                                //If mid

                                //If Left
                                .setReversed(false)
                                .splineTo(new Vector2d(-12, 34), Math.toRadians(0))

                                .build()
                );
        RoadRunnerBotEntity ComplexBlueRed = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, 60, Math.toRadians(-90)))

                                .splineTo(new Vector2d(36, 24),Math.toRadians(-90))
                                .splineTo(new Vector2d(24, 12),Math.toRadians(180))
                                .setReversed(true)
                                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(35, 12), Math.toRadians(180))

                                .setReversed(true)
                                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(35, 12), Math.toRadians(180))

                                .setReversed(true)
                                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(35, 12), Math.toRadians(180))

                                .setReversed(true)
                                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(35, 12), Math.toRadians(180))

                                .setReversed(true)
                                .splineTo(new Vector2d(56, 12), Math.toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(35, 12), Math.toRadians(180))
                                .strafeTo(new Vector2d(35,34))
                                //If Right
                                //.setReversed(false)
                                //.splineTo(new Vector2d(12, 34), Math.toRadians(180))
                                //If mid

                                //If Left
                                .setReversed(true)
                                .splineTo(new Vector2d(60, 34), Math.toRadians(0))

                                .build()
                );
        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                //.addEntity(BlueSideRedTerm)
                //.addEntity(BlueSideBlueTerm)
                //.addEntity(ComplexBlueBlue)
                .addEntity(ComplexBlueRed)
                .start();
    }
}