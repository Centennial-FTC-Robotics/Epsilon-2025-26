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
    private ServoEx turretYZ;

    private DcMotorEx flywheelMotor;
    private DcMotorEx intakeMotor;

    private ServoEx pusher;

    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        final double TURRET_POWER = 0.5;
        final double INTAKE_POWER = 1.0;
        final double MAX_ROTATION_POWER = 0.8;
        final double STICK_DEADZONE = 0.1; // stick drift

        final double DRIVE_MAX = 1.0;

        final double SHOOTER_POWER = 1.0;

        final double TURRET_YZ_MIN = 0.0;
        final double TURRET_YZ_MAX = 180.0;
        final double TURRET_YZ_STEP = 2.5;

        final double RETRACT_ANGLE = 180.0;
        final double PUSH_ANGLE = 6.7;

        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");

        turretXZ = hardwareMap.get(DcMotorEx.class, "turretXZ");
        turretYZ = new SimpleServo(hardwareMap, "turretYZ", 0, 180, AngleUnit.DEGREES);

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        pusher = new SimpleServo(hardwareMap, "pusher", 0, 180, AngleUnit.DEGREES);
        pusher.turnToAngle(RETRACT_ANGLE, AngleUnit.DEGREES);

        imu = hardwareMap.get(IMU.class, "imu"); // NOT USING IMU FOR NOW.

        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretXZ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretXZ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        while (!isStarted() && !isStopRequested()) {
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

            telemetry.addData("Pusher current (deg)", "%.1f", pusher.getAngle());
            telemetry.update();

            idle();
        }

        waitForStart();

        double turretYZAngle = 90.0;
        turretYZ.turnToAngle(turretYZAngle, AngleUnit.DEGREES);

        boolean aPrev1 = false;
        boolean aPrev2 = false;
        boolean yPrev1 = false;
        boolean yPrev2 = false;

        while (opModeIsActive()) {
            double lx1 = gamepad1.left_stick_x;
            double ly1 = -gamepad1.left_stick_y;
            double rx1 = gamepad1.right_stick_x;

            double lx2 = gamepad2.left_stick_x;
            double ly2 = -gamepad2.left_stick_y;
            double rx2 = gamepad2.right_stick_x;

            double lx = (Math.abs(lx1) > STICK_DEADZONE) ? lx1 : lx2;
            double ly = (Math.abs(ly1) > STICK_DEADZONE) ? ly1 : ly2;
            double rx = (Math.abs(rx1) > STICK_DEADZONE) ? rx1 : rx2;

            double rt = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
            double driveScale = 1.0 - (0.75 * rt);

            double rotationPower = 0.0;
            if (Math.abs(rx) > STICK_DEADZONE) {
                rotationPower = rx * MAX_ROTATION_POWER;
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

            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                turretCmdXZ = -TURRET_POWER;
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                turretCmdXZ = TURRET_POWER;
            } else {
                turretCmdXZ = 0.0;
            }
            turretXZ.setPower(turretCmdXZ);

            boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            boolean dpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadUp && !dpadDown) {
                turretYZAngle += TURRET_YZ_STEP;
                if (turretYZAngle > TURRET_YZ_MAX) turretYZAngle = TURRET_YZ_MAX;
                turretYZ.turnToAngle(turretYZAngle, AngleUnit.DEGREES);
            } else if (dpadDown && !dpadUp) {
                turretYZAngle -= TURRET_YZ_STEP;
                if (turretYZAngle < TURRET_YZ_MIN) turretYZAngle = TURRET_YZ_MIN;
                turretYZ.turnToAngle(turretYZAngle, AngleUnit.DEGREES);
            }

            double intakeCmd = 0.0;
            if (gamepad1.b || gamepad2.b) {
                intakeCmd = INTAKE_POWER;
            } else {
                intakeCmd = 0.0;
            }
            intakeMotor.setPower(intakeCmd);

            boolean aNow1 = gamepad1.a;
            boolean aNow2 = gamepad2.a;
            boolean aPressed = (aNow1 && !aPrev1) || (aNow2 && !aPrev2);
            if (aPressed) {
                // TODO: Implement shoot sequence here (start flywheel, pusher sequence, etc.)
            }
            aPrev1 = aNow1;
            aPrev2 = aNow2;

            boolean yNow1 = gamepad1.y;
            boolean yNow2 = gamepad2.y;
            boolean yPressed = (yNow1 && !yPrev1) || (yNow2 && !yPrev2);
            if (yPressed) {
                // TODO: Implement auto-aim / aim-at-target routine here
            }
            yPrev1 = yNow1;
            yPrev2 = yNow2;

            flywheelMotor.setPower(0.0);

            telemetry.addData("L stick", "lx=%.2f ly=%.2f", lx, ly);
            telemetry.addData("R stick rx", "%.2f", rx);
            telemetry.addData("Drive scale (rt)", "%.2f (rt=%.2f)", driveScale, rt);
            telemetry.addData("Motor calc", "FL=%.3f BL=%.3f FR=%.3f BR=%.3f", fl, bl, fr, br);
            telemetry.addData("Rotation Power", "%.3f", rotationPower);
            telemetry.addData("Turret XZ Power", "%.2f", turretCmdXZ);
            telemetry.addData("Turret YZ Angle", "%.1f", turretYZAngle);
            telemetry.addData("Intake Power", "%.2f", intakeCmd);
            telemetry.addData("Flywheel Power (idle)", "%.2f", 0.0);
            telemetry.update();

            idle();
        }
    }

    private double rangeClip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}