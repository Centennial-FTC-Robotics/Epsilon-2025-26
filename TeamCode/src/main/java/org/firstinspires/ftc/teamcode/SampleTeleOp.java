package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.ServoEx;
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
//    private DcMotorEx turretYZ;

    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;

    private ServoEx pusher;

    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        final double TURRET_POWER = 0.5;
        final double TURRET_SLOW_POWER = 0.25;
        final double INTAKE_POWER = 1.0;
        final double INTAKE_SLOW_POWER = 0.5;
        final double MAX_ROTATION_POWER = 0.8;
        final double ROTATION_KP = 2.0;
        final double STICK_DEADZONE = 0.1; // stick drift

        final double DRIVE_MAX = 1.0;
        final double DRIVE_SLOW_FACTOR = 0.25;
        final double ROTATE_SLOW_FACTOR = 0.5;

        final double SHOOTER_POWER = 1.0;
        final long SINGLE_SHOOT_TIME_MS = 500;
        final long INTER_SHOT_PAUSE_MS = 1000;
        final int MULTI_SHOT_COUNT = 3;

        final double RETRACT_ANGLE = 180.0;
        final double PUSH_ANGLE = 6.7;

        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");

        turretXZ = hardwareMap.get(DcMotorEx.class, "turretXZ");
//        turretYZ = hardwareMap.get(DcMotorEx.class, "turretYZ");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        pusher = new SimpleServo(hardwareMap, "pusher", 0, 180, AngleUnit.DEGREES);
        pusher.turnToAngle(RETRACT_ANGLE, AngleUnit.DEGREES);

        imu = hardwareMap.get(IMU.class, "imu"); // NOT USING IMU FOR NOW.

        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretXZ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        turretYZ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretXZ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turretYZ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // DO NOT MODIFY
        driveFL.setDirection(DcMotorEx.Direction.REVERSE);
        driveBL.setDirection(DcMotorEx.Direction.REVERSE);
        driveFR.setDirection(DcMotorEx.Direction.FORWARD);
        driveBR.setDirection(DcMotorEx.Direction.FORWARD);


        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        while (!isStarted() && !isStopRequested()) { // test individual motors for mapping (DO NOT MODIFY MAPPING FOR NOW)
            // Motor mapping test (unchanged behavior)
            telemetry.addLine("Motor test (press for test)");
            telemetry.addLine("A: FL  B: FR   X: BL   Y: BR");
            if (gamepad1.a) driveFL.setPower(0.5); else driveFL.setPower(0.0);
            if (gamepad1.b) driveFR.setPower(0.5); else driveFR.setPower(0.0);
            if (gamepad1.x) driveBL.setPower(0.5); else driveBL.setPower(0.0);
            if (gamepad1.y) driveBR.setPower(0.5); else driveBR.setPower(0.0);

            double movement = 0.0;
            if (gamepad1.dpad_down) movement = -0.1;
            if (gamepad1.dpad_up) movement = 0.1;
            pusher.rotateByAngle(movement, AngleUnit.DEGREES);

            // Telemetry to show what you're aiming for
            telemetry.addData("Pusher current (deg)", "%.1f", pusher.getAngle());
            telemetry.update();

            idle();
        }

        waitForStart();

        boolean aPrev1 = false;
        boolean aPrev2 = false;
        boolean xPrev1 = false;
        boolean xPrev2 = false;

        boolean isShootingSingle = false;
        long singleShootStartMs = 0;

        boolean isShootingMulti = false;
        int multiShotsFired = 0;
        boolean multiPhaseShooting = false;
        long multiPhaseStartMs = 0;

        while (opModeIsActive()) {
            double ly1 = -gamepad1.left_stick_y;
            double lx1 = gamepad1.left_stick_x;
            double rx1 = gamepad1.right_stick_x;
            double ry1 = -gamepad1.right_stick_y;

            double ly2 = -gamepad2.left_stick_y;
            double lx2 = gamepad2.left_stick_x;
            double rx2 = gamepad2.right_stick_x;
            double ry2 = -gamepad2.right_stick_y;

            double ly = (Math.abs(ly1) > STICK_DEADZONE) ? ly1 : ly2;
            double lx = (Math.abs(lx1) > STICK_DEADZONE) ? lx1 : lx2;
            double rx = (Math.abs(rx1) > STICK_DEADZONE) ? rx1 : rx2;
            double ry = (Math.abs(ry1) > STICK_DEADZONE) ? ry1 : ry2;
            double rMag = Math.abs(rx);

            boolean slowMode = gamepad1.y || gamepad2.y;
            double driveScale = slowMode ? DRIVE_SLOW_FACTOR : 1.0; // fix small naming if needed

            double rotationPower = 0.0;
            if (Math.abs(rx) > STICK_DEADZONE) {
                rotationPower = rx * MAX_ROTATION_POWER;
                if (slowMode) rotationPower *= ROTATE_SLOW_FACTOR;
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
            } else if (gamepad2.dpad_left) {
                turretCmdXZ = -TURRET_POWER;
            } else if (gamepad2.dpad_right) {
                turretCmdXZ = TURRET_POWER;
            }

            if (gamepad1.dpad_up) {
                turretCmdYZ = TURRET_POWER;
            } else if (gamepad1.dpad_down) {
                turretCmdYZ = -TURRET_POWER;
            } else if (gamepad2.dpad_up) {
                turretCmdYZ = TURRET_POWER;
            } else if (gamepad2.dpad_down) {
                turretCmdYZ = -TURRET_POWER;
            }

            if (gamepad1.b) {
                intakeCmd = 0.0;
            } else if (gamepad2.b) {
                intakeCmd = 0.0;
            } else {
                intakeCmd = INTAKE_POWER;
            }

            if (slowMode) {
                turretCmdXZ *= (TURRET_SLOW_POWER / TURRET_POWER);
                turretCmdYZ *= (TURRET_SLOW_POWER / TURRET_POWER);
                intakeCmd *= (INTAKE_SLOW_POWER / INTAKE_POWER);
            }

            turretXZ.setPower(turretCmdXZ);
//            turretYZ.setPower(turretCmdYZ);
            intakeMotor.setPower(intakeCmd);

            // --- NEW: shooter always runs unless B is pressed on either gamepad ---
            double shooterCmd;
            if (gamepad1.b || gamepad2.b) {
                shooterCmd = 0.0;
            } else {
                shooterCmd = -SHOOTER_POWER;
            }
            // apply shooter baseline power (shooting logic below will not change shooter power)
            shooterMotor.setPower(shooterCmd);

            long now = System.currentTimeMillis();

            boolean aNow1 = gamepad1.a;
            boolean aNow2 = gamepad2.a;
            boolean aPressedFrom1 = aNow1 && !aPrev1;
            boolean aPressedFrom2 = aNow2 && !aPrev2;

            if (aPressedFrom1) {
                if (!isShootingSingle && !isShootingMulti) {
                    isShootingSingle = true;
                    singleShootStartMs = now;
                    // shooterMotor.setPower(-SHOOTER_POWER); // no longer needed
                    pusher.turnToAngle(PUSH_ANGLE, AngleUnit.DEGREES);
                }
            } else if (aPressedFrom2) {
                if (!isShootingSingle && !isShootingMulti) {
                    isShootingSingle = true;
                    singleShootStartMs = now;
                    // shooterMotor.setPower(-SHOOTER_POWER); // no longer needed
                    pusher.turnToAngle(PUSH_ANGLE, AngleUnit.DEGREES);
                }
            }

            aPrev1 = aNow1;
            aPrev2 = aNow2;

            boolean xNow1 = gamepad1.x;
            boolean xNow2 = gamepad2.x;
            boolean xPressedFrom1 = xNow1 && !xPrev1;
            boolean xPressedFrom2 = xNow2 && !xPrev2;

            if (xPressedFrom1) {
                if (!isShootingMulti) {
                    // start multi-shoot
                    isShootingMulti = true;
                    multiShotsFired = 0;
                    multiPhaseShooting = true;
                    multiPhaseStartMs = now;
                    // shooterMotor.setPower(-SHOOTER_POWER); // no longer needed
                    pusher.turnToAngle(PUSH_ANGLE, AngleUnit.DEGREES);
                } else {
                    // cancel multi-shoot
                    isShootingMulti = false;
                    multiShotsFired = 0;
                    multiPhaseShooting = false;
                    // don't forcibly shut the shooter here; B controls shooter
                    pusher.turnToAngle(RETRACT_ANGLE, AngleUnit.DEGREES);
                }
            } else if (xPressedFrom2) {
                if (!isShootingMulti) {
                    isShootingMulti = true;
                    multiShotsFired = 0;
                    multiPhaseShooting = true;
                    multiPhaseStartMs = now;
                    // shooterMotor.setPower(-SHOOTER_POWER); // no longer needed
                    pusher.turnToAngle(PUSH_ANGLE, AngleUnit.DEGREES);
                } else {
                    isShootingMulti = false;
                    multiShotsFired = 0;
                    multiPhaseShooting = false;
                    // don't forcibly shut the shooter here; B controls shooter
                    pusher.turnToAngle(RETRACT_ANGLE, AngleUnit.DEGREES);
                }
            }

            xPrev1 = xNow1;
            xPrev2 = xNow2;

            if (isShootingSingle) {
                if (now - singleShootStartMs >= SINGLE_SHOOT_TIME_MS) {
                    // shooterMotor.setPower(0.0); // removed: shooter is controlled by B
                    isShootingSingle = false;

                    pusher.turnToAngle(RETRACT_ANGLE, AngleUnit.DEGREES);
                }
            }

            if (isShootingMulti) {
                if (multiPhaseShooting) {
                    if (now - multiPhaseStartMs >= SINGLE_SHOOT_TIME_MS) {
                        // shooterMotor.setPower(0.0); // removed
                        multiPhaseShooting = false;
                        multiPhaseStartMs = now;
                        multiShotsFired++;

                        pusher.turnToAngle(RETRACT_ANGLE, AngleUnit.DEGREES);
                    }
                } else {
                    if (now - multiPhaseStartMs >= INTER_SHOT_PAUSE_MS) {
                        if (multiShotsFired >= MULTI_SHOT_COUNT) {
                            isShootingMulti = false;
                            multiShotsFired = 0;
                            multiPhaseShooting = false;
                            // shooterMotor.setPower(0.0); // removed
                            pusher.turnToAngle(RETRACT_ANGLE, AngleUnit.DEGREES);
                        } else {
                            // shooterMotor.setPower(-SHOOTER_POWER); // not needed
                            multiPhaseShooting = true;
                            multiPhaseStartMs = now;

                            pusher.turnToAngle(PUSH_ANGLE, AngleUnit.DEGREES);
                        }
                    }
                }
            }

            telemetry.addData("Sticks", "lx=%.2f ly=%.2f", lx, ly);
            telemetry.addData("Motor calc", "FL=%.3f BL=%.3f FR=%.3f BR=%.3f", fl, bl, fr, br);
            telemetry.addData("Strafe", "x=%.2f y=%.2f", lx, ly);
            telemetry.addData("Right stick mag (rx)", "%.2f", rMag);
            telemetry.addData("Rotation Power", "%.3f", rotationPower);
            telemetry.addData("Turret XZ", "%.2f", turretCmdXZ);
            telemetry.addData("Turret YZ", "%.2f", turretCmdYZ);
            telemetry.addData("Intake Power", "%.2f", intakeCmd);
            telemetry.addData("Shooter Power (applied)", "%.2f", shooterCmd);
            telemetry.addData("Pusher angle (deg)", "%.1f", pusher.getAngle());
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