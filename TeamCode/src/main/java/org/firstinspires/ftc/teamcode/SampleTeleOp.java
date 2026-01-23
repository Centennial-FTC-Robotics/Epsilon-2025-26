package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

import org.firstinspires.ftc.teamcode.Actions.PusherAction;
import org.firstinspires.ftc.teamcode.Actions.TurretAction;
import org.firstinspires.ftc.teamcode.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.Actions.FlywheelAction;
import org.firstinspires.ftc.teamcode.Actions.ShootAction;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "TeleOpBlue")
public class SampleTeleOp extends OpMode {

    private DcMotorEx driveBL;
    private DcMotorEx driveBR;
    private DcMotorEx driveFL;
    private DcMotorEx driveFR;

    private DcMotorEx turretXZ;
    private ServoEx turretYZ;
    private DcMotorEx flywheelMotor;
    private DcMotorEx intakeMotor;
    private ServoEx pusher;

    private IMU imu;
    private MecanumDrive mecanum;

    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;
    private ButtonReader aReader1, aReader2;
    private ButtonReader yReader1, yReader2;
    private ButtonReader bReader1, bReader2;
    private ButtonReader dpadLeft1, dpadLeft2, dpadRight1, dpadRight2, dpadUp1, dpadUp2, dpadDown1, dpadDown2;

    // Actions
    private PusherAction pusherAction;
    private TurretAction turretAction;
    private IntakeAction intakeAction;
    private FlywheelAction flywheelAction;
    private ShootAction shootAction;

    // constants
    private final double TURRET_POWER = 0.5;
    private final double INTAKE_POWER = 1.0;
    private final double MAX_ROTATION_POWER = 0.8;
    private final double STICK_DEADZONE = 0.1;
    private final double DRIVE_MAX = 1.0;
    private final double SHOOTER_POWER = 1.0;
    private final double TURRET_YZ_MIN = 0.0;
    private final double TURRET_YZ_MAX = 180.0;
    private final double TURRET_YZ_STEP = 2.5;
    private final double RETRACT_ANGLE = 180.0;
    private final double PUSH_ANGLE = 6.7;

    private double turretYZAngle = 90.0; // default pitch target

    // Camera / AprilTag pipeline
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private final int ID_TAG_OF_INTEREST = 20; // 24 for RED
    private boolean autoAimActive = false;

    // Camera intrinsics from your sample (C920 @ 800x448). Update for your camera if needed.
    private final double fx = 578.272;
    private final double fy = 578.272;
    private final double cx = 402.145;
    private final double cy = 221.506;
    private final double tagSizeMeters = 16.5 / 100.0; // 16.5 cm -> 0.165 m

    // Auto-aim controller parameters
    private final double yawKp = 0.02;              // P gain for yaw -> turret XZ motor power
    private final double yawDeadbandDeg = 2.0;      // degrees tolerance for "on target"
    private final double maxTurretPower = TURRET_POWER;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // hardware
        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");

        turretXZ = hardwareMap. get(DcMotorEx.class, "turretXZ");
        turretYZ = new SimpleServo(hardwareMap, "turretYZ", 0, 180, AngleUnit.DEGREES);

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        pusher = new SimpleServo(hardwareMap, "pusher", 0, 180, AngleUnit.DEGREES);
        // pusherAction will set this to retract angle at construction

        imu = hardwareMap.get(BHI260IMU.class, "imu");

        // motor configuration
        driveBL.resetDeviceConfigurationForOpMode();
        driveBR.resetDeviceConfigurationForOpMode();
        driveFL.resetDeviceConfigurationForOpMode();
        driveFR.resetDeviceConfigurationForOpMode();

        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretXZ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretXZ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        mecanum = new MecanumDrive(true, driveFL, driveFR, driveBL, driveBR);

        turretYZAngle = 90.0;

        // gamepads
        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        aReader1 = new ButtonReader(gamepad1Ex, GamepadKeys.Button.A);
        aReader2 = new ButtonReader(gamepad2Ex, GamepadKeys.Button.A);

        yReader1 = new ButtonReader(gamepad1Ex, GamepadKeys.Button.Y);
        yReader2 = new ButtonReader(gamepad2Ex, GamepadKeys.Button.Y);

        bReader1 = new ButtonReader(gamepad1Ex, GamepadKeys.Button.B);
        bReader2 = new ButtonReader(gamepad2Ex, GamepadKeys.Button.B);

        dpadLeft1 = new ButtonReader(gamepad1Ex, GamepadKeys.Button.DPAD_LEFT);
        dpadLeft2 = new ButtonReader(gamepad2Ex, GamepadKeys.Button.DPAD_LEFT);

        dpadRight1 = new ButtonReader(gamepad1Ex, GamepadKeys.Button.DPAD_RIGHT);
        dpadRight2 = new ButtonReader(gamepad2Ex, GamepadKeys.Button.DPAD_RIGHT);

        dpadUp1 = new ButtonReader(gamepad1Ex, GamepadKeys.Button.DPAD_UP);
        dpadUp2 = new ButtonReader(gamepad2Ex, GamepadKeys.Button.DPAD_UP);

        dpadDown1 = new ButtonReader(gamepad1Ex, GamepadKeys.Button.DPAD_DOWN);
        dpadDown2 = new ButtonReader(gamepad2Ex, GamepadKeys.Button.DPAD_DOWN);

        // instantiate Actions (tune timings/powers to your hardware)
        pusherAction = new PusherAction(pusher, RETRACT_ANGLE, PUSH_ANGLE, 0.08, 0.12);
        turretAction = new TurretAction(turretXZ, turretYZ, TURRET_YZ_MIN, TURRET_YZ_MAX, TURRET_YZ_STEP, turretYZAngle);
        intakeAction = new IntakeAction(intakeMotor, INTAKE_POWER);
        flywheelAction = new FlywheelAction(flywheelMotor, 0.7, SHOOTER_POWER);
        shootAction = new ShootAction(flywheelAction, pusherAction, intakeAction, 3.0);

        // --- camera / pipeline setup ---
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "turretCam"), cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSizeMeters, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        readValues();

        // --- drive controls (unchanged) ---
        double lx1 = gamepad1Ex.getLeftX();
        double ly1 = gamepad1Ex.getLeftY();
        double rx1 = gamepad1Ex.getRightX();

        double lx2 = gamepad2Ex.getLeftX();
        double ly2 = gamepad2Ex.getLeftY();
        double rx2 = gamepad2Ex.getRightX();

        double lx = (Math.abs(lx1) > STICK_DEADZONE) ? lx1 : lx2;
        double ly = (Math.abs(ly1) > STICK_DEADZONE) ? ly1 : ly2;
        double rx = (Math.abs(rx1) > STICK_DEADZONE) ? rx1 : rx2;

        double rt1 = gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double rt2 = gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double rt = Math.max(rt1, rt2);
        double driveScale = 1.0 - (0.75 * rt);

        double rotationPower = (Math.abs(rx) > STICK_DEADZONE) ? rx * MAX_ROTATION_POWER : 0.0;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        mecanum.setMaxSpeed(driveScale);
        mecanum.driveFieldCentric(lx, ly, rotationPower, heading);

        // update Actions
        pusherAction.update();
        flywheelAction.update();
        turretAction.update();
        intakeAction.update();
        shootAction.update();

        // read dpad manual turret XZ input (we still allow it; auto-aim will override while active)
        boolean dpadLeft = dpadLeft1.isDown() || dpadLeft2.isDown();
        boolean dpadRight = dpadRight1.isDown() || dpadRight2.isDown();

        double turretCmdXZ = 0.0;
        if (dpadLeft) {
            turretCmdXZ = -TURRET_POWER;
        } else if (dpadRight) {
            turretCmdXZ = TURRET_POWER;
        } else {
            turretCmdXZ = 0.0;
        }

        // manual pitch control (only if auto-aim is NOT active)
        boolean dpadUp = dpadUp1.isDown() || dpadUp2.isDown();
        boolean dpadDown = dpadDown1.isDown() || dpadDown2.isDown();
        if (!autoAimActive) {
            if (dpadUp && !dpadDown) {
                turretAction.stepYZUp();
            } else if (dpadDown && !dpadUp) {
                turretAction.stepYZDown();
            }
        }

        // Intake: toggle on B press (wasJustPressed)
        if (bReader1.wasJustPressed() || bReader2.wasJustPressed()) {
            intakeAction.toggle();
            if (intakeAction.isBusy()) {
                flywheelAction.reverse();
            } else {
                flywheelAction.stop();
            }
        }

        // Shooting: A triggers coordinated single shot
        boolean aPressed = aReader1.wasJustPressed() || aReader2.wasJustPressed();
        if (aPressed) {
            shootAction.start();
            telemetry.addData("Event", "A pressed (shoot) triggered");
        }

        // Auto-aim toggle: Y toggles on/off
        boolean yPressed = yReader1.wasJustPressed() || yReader2.wasJustPressed();
        if (yPressed) {
            autoAimActive = !autoAimActive;
            telemetry.addData("AutoAim", autoAimActive ? "ENABLED" : "DISABLED");
        }

        // If auto-aim is active, attempt to find tag and aim
        double lastYawDeg = Double.NaN;
        double lastPitchDeg = Double.NaN;
        double lastServoAngle = Double.NaN;
        double lastTurretPower = 0.0;
        boolean sawTarget = false;

        if (autoAimActive) {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();
            if (detections != null && !detections.isEmpty()) {
                // find tag ID of interest
                AprilTagDetection best = null;
                for (AprilTagDetection d : detections) {
                    if (d.id == ID_TAG_OF_INTEREST) {
                        best = d;
                        break;
                    }
                }

                if (best != null) {
                    sawTarget = true;

                    // AprilTag pose (units = meters): best.pose.x, best.pose.y, best.pose.z
                    // Coordinate assumptions (OpenFTC/APRILTAG): x -> forward, y -> left, z -> up
                    // yaw (radians) = atan2(y, x)   (positive => target is to the left)
                    double tx = best.pose.x;
                    double ty = best.pose.y;
                    double tz = best.pose.z;

                    double yawRad = Math.atan2(ty, tx); // positive left
                    double yawDeg = Math.toDegrees(yawRad);
                    lastYawDeg = yawDeg;

                    // pitch: elevation angle above camera forward
                    // pitchRad = atan2(z, sqrt(x^2 + y^2))
                    double horizDist = Math.hypot(tx, ty);
                    double pitchRad = Math.atan2(tz, horizDist); // positive => tag is above
                    double pitchDeg = Math.toDegrees(pitchRad);
                    lastPitchDeg = pitchDeg;

                    // Convert pitch to turretYZ servo angle.
                    // We assume 90 deg corresponds to "camera/turret forward horizontal".
                    // If pitch is positive (tag above), we subtract pitch from 90 to point up.
                    double servoAngle = 90.0 - pitchDeg;
                    // clamp to allowed range:
                    if (servoAngle < TURRET_YZ_MIN) servoAngle = TURRET_YZ_MIN;
                    if (servoAngle > TURRET_YZ_MAX) servoAngle = TURRET_YZ_MAX;
                    lastServoAngle = servoAngle;

                    // Command pitch servo
                    turretAction.setYZAngle(servoAngle);

                    // Command turret XZ with a P controller using yaw error (we want yaw -> 0)
                    double yawErrorDeg = yawDeg; // positive => target is left, so set positive power to rotate left
                    double power = yawKp * yawErrorDeg;
                    // enforce minimal deadband and clamp
                    if (Math.abs(yawErrorDeg) <= yawDeadbandDeg) {
                        // on target
                        turretAction.stopXZ();
                        lastTurretPower = 0.0;
                    } else {
                        // clamp
                        if (power > maxTurretPower) power = maxTurretPower;
                        if (power < -maxTurretPower) power = -maxTurretPower;
                        turretAction.rotateXZ(power);
                        lastTurretPower = power;
                    }
                }
            }

            // If we didn't see the tag, stop turret motion to avoid runaway
            if (!sawTarget) {
                turretAction.stopXZ();
            }
        } else {
            // auto-aim inactive -> use manual XZ power from dpad
            turretAction.rotateXZ(turretCmdXZ);
        }

        // telemetry
        telemetry.addData("AutoAim", autoAimActive ? "ON" : "OFF");
        telemetry.addData("Saw target", sawTarget ? "YES" : "NO");
        if (sawTarget) {
            telemetry.addData("Tag yaw (deg)", "%.2f", Double.isNaN(lastYawDeg) ? 0.0 : lastYawDeg);
            telemetry.addData("Tag pitch (deg)", "%.2f", Double.isNaN(lastPitchDeg) ? 0.0 : lastPitchDeg);
            telemetry.addData("Turret servo angle", "%.2f", Double.isNaN(lastServoAngle) ? turretAction.getYZAngle() : lastServoAngle);
            telemetry.addData("Turret XZ power", "%.3f", lastTurretPower);
        }
        telemetry.addData("L stick", "lx=%.2f ly=%.2f", lx, ly);
        telemetry.addData("R stick rx", "%.2f", rx);
        telemetry.addData("Drive scale (rt)", "%.2f (rt=%.2f)", driveScale, rt);
        telemetry.addData("Rotation Power", "%.3f", rotationPower);
        telemetry.addData("Intake Power", "%.2f", intakeAction.getPower());
        telemetry.addData("Flywheel Power", "%.2f", flywheelMotor.getPower());
        telemetry.addData("Turret YZ Angle (target)", "%.1f", turretAction.getYZAngle());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.update();
    }

    @Override
    public void stop() {
        mecanum.stop();

        // stop & reset Actions
        turretAction.stop();
        intakeAction.stop();
        flywheelAction.stop();
        pusherAction.stop();
        shootAction.stop();

        turretXZ.setPower(0.0);
        intakeMotor.setPower(0.0);
        flywheelMotor.setPower(0.0);

        // stop camera streaming (defensive)
        if (camera != null) {
            try {
                camera.stopStreaming();
            } catch (Exception ignored) {}
        }
    }

    private void readValues() {
        aReader1.readValue();
        aReader2.readValue();
        yReader1.readValue();
        yReader2.readValue();
        bReader1.readValue();
        bReader2.readValue();

        dpadLeft1.readValue();
        dpadLeft2.readValue();
        dpadRight1.readValue();
        dpadRight2.readValue();
        dpadUp1.readValue();
        dpadUp2.readValue();
        dpadDown1.readValue();
        dpadDown2.readValue();
    }
}