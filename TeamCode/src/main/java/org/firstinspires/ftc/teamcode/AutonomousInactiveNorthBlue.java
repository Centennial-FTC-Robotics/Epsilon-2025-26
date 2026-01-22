package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutonomousInactiveNorthBlue", group = "Auto")
public class AutonomousInactiveNorthBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double t = 23.5;

        Pose2d startPose = new Pose2d(
                t * -2,
                t * -2,
                Math.toRadians(0)
        );

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence seq = drive.trajectorySequenceBuilder(startPose)
//                .waitSeconds(3.0)
                .splineToConstantHeading(
                        new Vector2d(
                                t * -1.25,
                                t * -2.25
                        ),
                        Math.toRadians(0)
                )
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(seq);

        while (opModeIsActive()) {
            idle();
        }
    }
}