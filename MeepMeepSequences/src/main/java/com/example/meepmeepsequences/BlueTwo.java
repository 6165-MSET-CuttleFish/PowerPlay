package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueTwo {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(58, 35, Math.toRadians(180)))
                                //initial setup from starting point: 1
                                .splineToConstantHeading(new Vector2d(35, 35), Math.toRadians(180))
                                .strafeTo(new Vector2d(35, 0))
                                .addDisplacementMarker(() -> {
                                   //robot.score();
                                    //TWO SECONDS?
                                })
                                .waitSeconds(2)
                                .splineTo(new Vector2d(57, 12), Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    //robot.pickup();
                                    //ONE SECOND?
                                })
                                .waitSeconds(1)

                                //can repeat (cycle) from here: 1 + 1
                                .lineToConstantHeading(new Vector2d(23, 12))
                                .addDisplacementMarker(() -> {
                                    //robot.score();
                                    //TWO SECONDS?
                                })
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(57, 12))
                                .addDisplacementMarker(() -> {
                                    //robot.pickup();
                                    //ONE SECOND?
                                })
                                .waitSeconds(1)

                                //1 + 2
                                .lineToConstantHeading(new Vector2d(23, 12))
                                .addDisplacementMarker(() -> {
                                    //robot.score();
                                    //TWO SECONDS?
                                })
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(57, 12))
                                .addDisplacementMarker(() -> {
                                    //robot.pickup();
                                    //ONE SECOND?
                                })
                                .waitSeconds(1)

                                //1 + 3
                                .lineToConstantHeading(new Vector2d(23, 12))
                                .addDisplacementMarker(() -> {
                                    //robot.score();
                                    //TWO SECONDS?
                                })
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(57, 12))
                                .addDisplacementMarker(() -> {
                                    //robot.pickup();
                                    //ONE SECOND?
                                })
                                .waitSeconds(1)
                                //1 + 4
                                .lineToConstantHeading(new Vector2d(23, 12))
                                .addDisplacementMarker(() -> {
                                    //robot.score();
                                    //TWO SECONDS?
                                })
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(57, 12))
                                .addDisplacementMarker(() -> {
                                    //robot.pickup();
                                    //ONE SECOND?
                                })
                                .waitSeconds(1)
                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
