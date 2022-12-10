package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.*;

import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Declare a MeepMeep instance

        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(550);

        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(38,-61,Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(34.2, -21.8))
                                .lineToConstantHeading(new Vector2d(40, -8.0))
                                .lineToConstantHeading(new Vector2d(40, -12.3))
                                .lineToLinearHeading(new Pose2d(41, -12.3, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(63.20,-12.3))
                                .lineToConstantHeading(new Vector2d(25.5,-12.7))
                                .lineToConstantHeading(new Vector2d(45, -12.3))
                                .lineToConstantHeading(new Vector2d(63.25,-12.3))
                                .lineToConstantHeading(new Vector2d(25.5,-12.7))
                                .lineToConstantHeading(new Vector2d(45, -12.3))
                                .lineToConstantHeading(new Vector2d(63.25,-12.3))
                                .lineToConstantHeading(new Vector2d(25.5,-12.7))
                                .lineToConstantHeading(new Vector2d(45, -12.3))
                                .lineToConstantHeading(new Vector2d(63.25,-12.3))
                                .lineToConstantHeading(new Vector2d(38,-11.98))
                                .build()




                );
        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38,-61,Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(-34.2, -21.8))
                                .lineToConstantHeading(new Vector2d(-40, -8.0))
                                .lineToConstantHeading(new Vector2d(-40, -12.3))
                                .lineToLinearHeading(new Pose2d(-41, -12.3, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(-63.20,-12.3))
                                .lineToConstantHeading(new Vector2d(-25.5,-12.7))
                                .lineToConstantHeading(new Vector2d(-45, -12.3))
                                .lineToConstantHeading(new Vector2d(-63.25,-12.3))
                                .lineToConstantHeading(new Vector2d(-25.5,-12.7))
                                .lineToConstantHeading(new Vector2d(-45, -12.3))
                                .lineToConstantHeading(new Vector2d(-63.25,-12.3))
                                .lineToConstantHeading(new Vector2d(-25.5,-12.7))
                                .lineToConstantHeading(new Vector2d(-45, -12.3))
                                .lineToConstantHeading(new Vector2d(-63.25,-12.3))
                                .lineToConstantHeading(new Vector2d(-38,-11.98))
                                .build()




                );
        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .addEntity(blueLeft)
                .addEntity(new RingEntity(meepMeep,new Vector2d(64.7,-14.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(64.7,-12.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(64.7,-10.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(24,-14.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(24,-12.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(24,-10.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(34.2, -21.8),new Vector2d(0,0)))

                .addEntity(new RingEntity(meepMeep,new Vector2d(-64.7,-14.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(-64.7,-12.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(-64.7,-10.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(-24,-14.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(-24,-12.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(-24,-10.5),new Vector2d(0,0) ))
                .addEntity(new RingEntity(meepMeep,new Vector2d(-34.2, -21.8),new Vector2d(0,0) ))
                .start();
    }
}