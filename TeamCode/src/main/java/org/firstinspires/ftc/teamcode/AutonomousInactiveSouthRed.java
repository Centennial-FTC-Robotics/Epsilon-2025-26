package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutonomousInactiveSouthRed", group = "Auto")
public class AutonomousInactiveSouthRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        double t = 23.5;

        // Start pose (matches MeepMeep exactly)
        Pose2d startPose = new Pose2d(
                t * 2.55,
                t * 0.5,
                Math.toRadians(180)
        );

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence seq = drive.trajectorySequenceBuilder(startPose)
//                .waitSeconds(3.0)
                .splineToConstantHeading(
                        new Vector2d(
                                t * 1.5,
                                t * 0.5
                        ),
                        Math.toRadians(180)
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
