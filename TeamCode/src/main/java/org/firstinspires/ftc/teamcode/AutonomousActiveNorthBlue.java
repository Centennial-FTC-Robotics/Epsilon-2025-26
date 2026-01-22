package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Actions.FlywheelAction;
import org.firstinspires.ftc.teamcode.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.Actions.PusherAction;
import org.firstinspires.ftc.teamcode.Actions.ShootAction;
import org.firstinspires.ftc.teamcode.Actions.TurretAction;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "ActiveNorthBlue", group = "Auto")
public class AutonomousActiveNorthBlue extends LinearOpMode {

    // ---- constants ----
    private static final double TILE = 23.5;

    // Actions
    private FlywheelAction flywheelAction;
    private IntakeAction intakeAction;
    private PusherAction pusherAction;
    private TurretAction turretAction;
    private ShootAction shootAction;

    private SampleMecanumDrive drive;

    // vision / auto-aim
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagPipeline;
    private boolean autoAimActive = false;

    @Override
    public void runOpMode() {

        // ---- RoadRunner ----
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(
                TILE * -2,
                TILE * -2,
                Math.toRadians(0)
        );
        drive.setPoseEstimate(startPose);

        // ---- hardware ----
        DcMotorEx turretXZ = hardwareMap.get(DcMotorEx.class, "turretXZ");
        ServoEx turretYZ = new SimpleServo(hardwareMap, "turretYZ", 0, 180, AngleUnit.DEGREES);
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        ServoEx pusher = new SimpleServo(hardwareMap, "pusher", 0, 180, AngleUnit.DEGREES);

        // ---- Actions ----
        pusherAction = new PusherAction(pusher, 180.0, 6.7, 0.08, 0.12);
        turretAction = new TurretAction(turretXZ, turretYZ, 0, 180, 2.5, 90);
        intakeAction = new IntakeAction(intake, 1.0);
        flywheelAction = new FlywheelAction(flywheel, 0.7, 1.0);
        shootAction = new ShootAction(flywheelAction, pusherAction, intakeAction, 3.0);

        // ---- vision ----
        setupAprilTags();

        waitForStart();
        if (isStopRequested()) return;

        /* ===================== TRAJECTORY ===================== */

        Action driveAction = drive.actionBuilder(startPose)
                .waitSeconds(3.0)

                .splineToConstantHeading(
                        new Vector2d(TILE * -2, TILE * -0.5),
                        Math.toRadians(-180)
                )

                .turnTo(Math.toRadians(-115))

                // ---- SHOOTING ZONE ----
                .waitSeconds(0.1) // just to settle
                .build();

        drive.runAction(driveAction);

        /* ===================== SHOOT ===================== */

        autoAimActive = true;
        flywheelAction.spinUp();

        while (opModeIsActive() && !flywheelAction.isAtSpeed()) {
            updateAll();
            autoAim();
        }

        shootAction.start();

        while (opModeIsActive() && shootAction.isBusy()) {
            updateAll();
            autoAim();
        }

        autoAimActive = false;
        flywheelAction.stopFlywheel();

        /* ===================== CONTINUE PATH ===================== */

        Action rest = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(
                        new Pose2d(TILE * -0.5, TILE * -1.5, Math.toRadians(-90)),
                        Math.toRadians(-90)
                )
                .splineToConstantHeading(
                        new Vector2d(TILE * -0.5, TILE * -2.25),
                        Math.toRadians(-90)
                )
                .waitSeconds(2.0)
                .lineToYLinearHeading(TILE * -2, Math.toRadians(-90))
                .lineToYLinearHeading(TILE * -1.5, Math.toRadians(-270))
                .build();

        drive.runAction(rest);

        // stop everything
        stopAll();
    }

    /* ===================== HELPERS ===================== */

    private void updateAll() {
        turretAction.update();
        intakeAction.update();
        flywheelAction.update();
        pusherAction.update();
        shootAction.update();
    }

    private void stopAll() {
        turretAction.stop();
        intakeAction.stop();
        flywheelAction.stop();
        pusherAction.stop();
        shootAction.stop();
    }

    private void autoAim() {
        if (!autoAimActive) return;

        var detections = aprilTagPipeline.getLatestDetections();
        if (detections == null) return;

        for (var tag : detections) {
            if (tag.id == 20) {
                double yaw = Math.atan2(tag.pose.y, tag.pose.x);
                double pitch = Math.atan2(tag.pose.z,
                        Math.hypot(tag.pose.x, tag.pose.y));

                turretAction.rotateXZ(0.02 * Math.toDegrees(yaw));
                turretAction.setYZAngle(90.0 - Math.toDegrees(pitch));
                return;
            }
        }
        turretAction.stopXZ();
    }

    private void setupAprilTags() {
        int id = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "turretCam"), id);

        aprilTagPipeline = new AprilTagDetectionPipeline(
                0.165, 578.272, 578.272, 402.145, 221.506);

        camera.setPipeline(aprilTagPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // resolution should match what the pipeline was calibrated for; sample used 800x448
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "open error: " + errorCode);
                telemetry.update();
            }
        });
    }
}
