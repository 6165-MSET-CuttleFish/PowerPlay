package com.example.meepmeepsequences.Kai;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.*;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class Kai {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 600 pixels
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity lsopydil = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, 60, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(36,12))
                                .lineToConstantHeading(new Vector2d(60,12))
                                .lineToConstantHeading(new Vector2d(0,12))
                                .lineToConstantHeading(new Vector2d(12,12))
                                .lineToConstantHeading(new Vector2d(12,34))
                                .lineToConstantHeading(new Vector2d(59,34))
                                .build()
                );
        RoadRunnerBotEntity lsopydil2 = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-36,12))
                                .lineToConstantHeading(new Vector2d(-60,12))
                                .lineToConstantHeading(new Vector2d(0,12))
                                .lineToConstantHeading(new Vector2d(-60,12))
                                .lineToConstantHeading(new Vector2d(0,12))
                                .lineToConstantHeading(new Vector2d(-60,12))
                                .lineToConstantHeading(new Vector2d(0,12))
                                .lineToConstantHeading(new Vector2d(-12,12))
                                .lineToConstantHeading(new Vector2d(-12,34))
                                .lineToConstantHeading(new Vector2d(-59,34))
                                .build()
                );
        RoadRunnerBotEntity lsopydil3 = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -60, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(36,-12))
                                .lineToConstantHeading(new Vector2d(60,-12))
                                .lineToConstantHeading(new Vector2d(0,-12))
                                .lineToConstantHeading(new Vector2d(60,-12))
                                .lineToConstantHeading(new Vector2d(0,-12))
                                .lineToConstantHeading(new Vector2d(60,-12))
                                .lineToConstantHeading(new Vector2d(0,-12))
                                .lineToConstantHeading(new Vector2d(60,-12))
                                .lineToConstantHeading(new Vector2d(0,-12))
                                .lineToConstantHeading(new Vector2d(12,-12))
                                .lineToConstantHeading(new Vector2d(12,-34))
                                .lineToConstantHeading(new Vector2d(59,-34))
                                .build()
                );
        RoadRunnerBotEntity lsopydil4 = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-36,-12))
                                .lineToConstantHeading(new Vector2d(-60,-12))
                                .lineToConstantHeading(new Vector2d(0,-12))
                                .lineToConstantHeading(new Vector2d(-60,-12))
                                .lineToConstantHeading(new Vector2d(0,-12))
                                .lineToConstantHeading(new Vector2d(-60,-12))
                                .lineToConstantHeading(new Vector2d(0,-12))
                                .lineToConstantHeading(new Vector2d(-60,-12))
                                .lineToConstantHeading(new Vector2d(0,-12))
                                .lineToConstantHeading(new Vector2d(-60,-12))
                                .lineToConstantHeading(new Vector2d(0,-12))
                                .lineToConstantHeading(new Vector2d(-12,-12))
                                .lineToConstantHeading(new Vector2d(-12,-34))
                                .lineToConstantHeading(new Vector2d(-59,-34))
                                .build()
                );
        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .addEntity(lsopydil)
                .addEntity(lsopydil2)
                .addEntity(lsopydil3)
                .addEntity(lsopydil4)
                .setBackgroundAlpha(0.95f)
                .start();
    }
}
