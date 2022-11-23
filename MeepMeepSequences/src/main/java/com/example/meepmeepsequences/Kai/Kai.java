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

        RoadRunnerBotEntity lsopydil = new DefaultBotBuilder(meepMeep).setDimensions(16,17)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, 62, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(34,23,Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(34,12))
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(59,12))
                                .lineToConstantHeading(new Vector2d(23.5,14))
                                .lineToConstantHeading(new Vector2d(59,12))
                                .lineToConstantHeading(new Vector2d(23.5,14))
                                .lineToConstantHeading(new Vector2d(59,12))
                                .lineToConstantHeading(new Vector2d(23.5,14))
                                .lineToConstantHeading(new Vector2d(59,12))
                                .lineToConstantHeading(new Vector2d(23.5,14))
                                .lineToConstantHeading(new Vector2d(59,12))
                                .lineToConstantHeading(new Vector2d(23.5,14))
                                .lineToConstantHeading(new Vector2d(0,14))

                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .addEntity(lsopydil)
                .setBackgroundAlpha(0.95f)
                .start();
    }
}
