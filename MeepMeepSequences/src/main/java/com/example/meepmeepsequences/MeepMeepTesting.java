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
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(0)))

                                //LEFT

                                /*.strafeRight(50)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .strafeLeft(23)*/

                                .lineToSplineHeading(new Pose2d(-35.5, 12, Math.toRadians(180)))
                                //.lineToLinearHeading(new Pose2d(-35.5, 12, Math.toRadians(180)))
                                //.setReversed(true)
                               // .splineToLinearHeading(new Pose2d(-35.5, 12), Math.toRadians(180))

                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)

                                .strafeRight(23)
                                .forward(20)

                                .build()




                );

        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 62, Math.toRadians(0)))
                                .strafeRight(50)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .strafeLeft(23)

                                /*.lineToSplineHeading(new Pose2d(35.5, 12, Math.toRadians(180)))
                                //.lineToLinearHeading(new Pose2d(-35.5, 12, Math.toRadians(180)))
                                //.setReversed(true)
                                // .splineToLinearHeading(new Pose2d(-35.5, 12), Math.toRadians(180))

                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)

                                .strafeRight(23)
                                .forward(20)*/

                                .build()
                );

        RoadRunnerBotEntity redRight = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -62, Math.toRadians(0)))
                                /*.strafeLeft(50)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .strafeRight(23)*/

                                .lineToSplineHeading(new Pose2d(35.5, -12, Math.toRadians(180)))
                                //.lineToLinearHeading(new Pose2d(-35.5, 12, Math.toRadians(180)))
                                //.setReversed(true)
                                // .splineToLinearHeading(new Pose2d(-35.5, 12), Math.toRadians(180))

                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)
                                .back(21.5)

                                .forward(21.5)

                                .strafeLeft(23)
                                .forward(20)

                                .build()
                );

        RoadRunnerBotEntity redLeft = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(0)))

                                //LEFT

                                .strafeLeft(50)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .strafeRight(23)

                                /*.lineToSplineHeading(new Pose2d(-35.5, -12, Math.toRadians(180)))
                                //.lineToLinearHeading(new Pose2d(-35.5, 12, Math.toRadians(180)))
                                //.setReversed(true)
                                // .splineToLinearHeading(new Pose2d(-35.5, 12), Math.toRadians(180))

                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)
                                .forward(21.5)

                                .back(21.5)

                                .strafeLeft(23)
                                .forward(20)*/

                                .build()




                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(blueLeft)
                .addEntity(blueRight)
                .addEntity(redRight)
                .addEntity(redLeft)
                .start();
    }
}