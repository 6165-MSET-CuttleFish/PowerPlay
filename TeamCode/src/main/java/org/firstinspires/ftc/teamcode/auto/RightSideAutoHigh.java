package org.firstinspires.ftc.teamcode.auto;

public class RightSideAutoHigh {
     /*
        Trajectory fastPreload1 = robot.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-11,56, Math.toRadians(180)))
                .build();
        Trajectory preloadInit = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-39,56))
                .build();
        Trajectory preload1 = robot.trajectoryBuilder(new Pose2d(-39,56,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-11, 56))
                .build();

        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                .lineToConstantHeading(new Vector2d(-11, 19))
                .addDisplacementMarker(2, ()->{
                            slides.setState(Slides.State.HIGH);
                            fourbar.setState(vfourb.State.ALIGN_POSITION);
                        }
                )
                .build();
        Trajectory preload3 = robot.trajectoryBuilder(preload2.end())

                .lineToConstantHeading(new Vector2d(-9,19), robot.getVelocityConstraint(10, 5.939, 14.48),
                        robot.getAccelerationConstraint(45))
                .addTemporalMarker(0.25,()->{
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);

                })
                .addTemporalMarker(2.5, ()->{
                    intake.setState(Intake.State.DEPOSITING);
                })

                .build();
        Trajectory preload4 = robot.trajectoryBuilder(preload3.end())
                .lineToConstantHeading(new Vector2d(-11,11))
                .addDisplacementMarker(2,()->{
                    fourbar.setState(vfourb.State.PRIMED);
                    intake.setState(Intake.State.OFF);
                    slides.setState(Slides.State.BOTTOM);
                })
                .build();

        */
}
