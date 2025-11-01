package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "SampleTeleOp")
public class SampleTeleOp extends LinearOpMode {

    private DcMotorEx driveBL;
    private DcMotorEx driveBR;
    private DcMotorEx driveFL;
    private DcMotorEx driveFR;

    private DcMotorEx turretXZ;
    private DcMotorEx turretYZ;

    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;

    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");

        turretXZ = hardwareMap.get(DcMotorEx.class, "turretXZ");
        turretYZ = hardwareMap.get(DcMotorEx.class, "turretYZ");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        imu = hardwareMap.get(IMU.class, "imu");

        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretXZ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretYZ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretXZ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretYZ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveBL.setDirection(DcMotorEx.Direction.REVERSE); // flip

        // make sure it is in position so it can be calculated correctlyf
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        final double TURRET_POWER = 0.5;
        final double TURRET_SLOW_POWER = 0.25;
        final double INTAKE_POWER = 1.0;
        final double INTAKE_SLOW_POWER = 0.5;
        final double MAX_ROTATION_POWER = 0.8;
        final double ROTATION_KP = 2.0;
        final double STICK_DEADZONE = 0.1; // stick drift

        final double DRIVE_MAX = 1.0;
        final double DRIVE_SLOW_FACTOR = 0.5;

        final double SHOOTER_POWER = 1.0;
        final long SINGLE_SHOOT_TIME_MS = 500;
        final long INTER_SHOT_PAUSE_MS = 1000;
        final int MULTI_SHOT_COUNT = 3;

        boolean aPrev = false;
        boolean xPrev = false;

        boolean isShootingSingle = false;
        long singleShootStartMs = 0;

        boolean isShootingMulti = false;
        int multiShotsFired = 0;
        boolean multiPhaseShooting = false;
        long multiPhaseStartMs = 0;

        while (opModeIsActive()) {
            double ly = -gamepad1.left_stick_y;
            double lx = gamepad1.left_stick_x;

            double rx = gamepad1.right_stick_x;
            double ry = -gamepad1.right_stick_y;
            double rMag = Math.hypot(rx, ry);

            boolean slowMode = gamepad1.y;
            double driveScale = slowMode ? DRIVE_SLOW_FACTOR : 1.0;

            double rotationPower = 0.0;
            if (rMag > STICK_DEADZONE) {
                double desiredHeading = Math.atan2(rx, ry);
                double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double error = desiredHeading - currentHeading;
                while (error > Math.PI) error -= 2.0 * Math.PI;
                while (error < -Math.PI) error += 2.0 * Math.PI;

                rotationPower = ROTATION_KP * error;
                if (rotationPower > MAX_ROTATION_POWER) rotationPower = MAX_ROTATION_POWER;
                if (rotationPower < -MAX_ROTATION_POWER) rotationPower = -MAX_ROTATION_POWER;

                if (slowMode) rotationPower *= DRIVE_SLOW_FACTOR;
            } else {
                rotationPower = 0.0;
            }

            double denom = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rotationPower), 1.0);
            double fl = (ly + lx + rotationPower) / denom;
            double bl = (ly - lx + rotationPower) / denom;
            double fr = (ly - lx - rotationPower) / denom;
            double br = (ly + lx - rotationPower) / denom;

            fl *= driveScale;
            bl *= driveScale;
            fr *= driveScale;
            br *= driveScale;

            driveFL.setPower(rangeClip(fl, -DRIVE_MAX, DRIVE_MAX));
            driveBL.setPower(rangeClip(bl, -DRIVE_MAX, DRIVE_MAX));
            driveFR.setPower(rangeClip(fr, -DRIVE_MAX, DRIVE_MAX));
            driveBR.setPower(rangeClip(br, -DRIVE_MAX, DRIVE_MAX));

            double turretCmdXZ = 0.0;
            double turretCmdYZ = 0.0;
            double intakeCmd = 0.0;

            if (gamepad1.dpad_left) {
                turretCmdXZ = -TURRET_POWER;
            } else if (gamepad1.dpad_right) {
                turretCmdXZ = TURRET_POWER;
            }

            if (gamepad1.dpad_up) {
                turretCmdYZ = TURRET_POWER;
            } else if (gamepad1.dpad_down) {
                turretCmdYZ = -TURRET_POWER;
            }

            if (gamepad1.b) {
                intakeCmd = INTAKE_POWER;
            }

            if (slowMode) {
                turretCmdXZ *= (TURRET_SLOW_POWER / TURRET_POWER);
                turretCmdYZ *= (TURRET_SLOW_POWER / TURRET_POWER);
                intakeCmd *= (INTAKE_SLOW_POWER / INTAKE_POWER);
            }

            turretXZ.setPower(turretCmdXZ);
            turretYZ.setPower(turretCmdYZ);
            intakeMotor.setPower(intakeCmd);

            long now = System.currentTimeMillis();

            if (gamepad1.a && !aPrev) {
                if (!isShootingSingle && !isShootingMulti) {
                    isShootingSingle = true;
                    singleShootStartMs = now;
                    shooterMotor.setPower(SHOOTER_POWER);
                }
            }
            aPrev = gamepad1.a;

            if (gamepad1.x && !xPrev) {
                if (!isShootingMulti) {
                    isShootingMulti = true;
                    multiShotsFired = 0;
                    multiPhaseShooting = true;
                    multiPhaseStartMs = now;
                    shooterMotor.setPower(SHOOTER_POWER);
                } else {
                    isShootingMulti = false;
                    multiShotsFired = 0;
                    multiPhaseShooting = false;
                    shooterMotor.setPower(0.0);
                }
            }
            xPrev = gamepad1.x;

            if (isShootingSingle) {
                if (now - singleShootStartMs >= SINGLE_SHOOT_TIME_MS) {
                    shooterMotor.setPower(0.0);
                    isShootingSingle = false;
                }
            }

            if (isShootingMulti) {
                if (multiPhaseShooting) {
                    if (now - multiPhaseStartMs >= SINGLE_SHOOT_TIME_MS) {
                        shooterMotor.setPower(0.0);
                        multiPhaseShooting = false;
                        multiPhaseStartMs = now;
                        multiShotsFired++;
                    }
                } else {
                    if (now - multiPhaseStartMs >= INTER_SHOT_PAUSE_MS) {
                        if (multiShotsFired >= MULTI_SHOT_COUNT) {
                            isShootingMulti = false;
                            multiShotsFired = 0;
                            multiPhaseShooting = false;
                            shooterMotor.setPower(0.0);
                        } else {
                            shooterMotor.setPower(SHOOTER_POWER);
                            multiPhaseShooting = true;
                            multiPhaseStartMs = now;
                        }
                    }
                }
            }

            if (!isShootingSingle && !isShootingMulti) {
                shooterMotor.setPower(0.0);
            }

            telemetry.addData("Drive (L stick)", "x=%.2f y=%.2f", lx, ly);
            telemetry.addData("Right stick mag", "%.2f", rMag);
            telemetry.addData("Rotation Power", "%.3f", rotationPower);
            telemetry.addData("Turret XZ", "%.2f", turretCmdXZ);
            telemetry.addData("Turret YZ", "%.2f", turretCmdYZ);
            telemetry.addData("Intake Power", "%.2f", intakeCmd);
            telemetry.addData("Shooting Single", isShootingSingle);
            telemetry.addData("Shooting Multi", isShootingMulti);
            telemetry.addData("Multi Fired", multiShotsFired);
            telemetry.update();

            idle();
        }
    }

    private double rangeClip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}