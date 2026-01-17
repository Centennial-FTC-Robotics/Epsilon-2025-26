package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "SampleTeleOp")
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

    // constants
    private final double TURRET_POWER = 0.5;
    private final double INTAKE_POWER = 1.0;
    private final double MAX_ROTATION_POWER = 0.8;
    private final double STICK_DEADZONE = 0.1; // stick drift
    private final double DRIVE_MAX = 1.0;
    private final double SHOOTER_POWER = 1.0;
    private final double TURRET_YZ_MIN = 0.0;
    private final double TURRET_YZ_MAX = 180.0;
    private final double TURRET_YZ_STEP = 2.5;
    private final double RETRACT_ANGLE = 180.0;
    private final double PUSH_ANGLE = 6.7;

    private double turretYZAngle = 90.0;

    private boolean aPrev1 = false;
    private boolean aPrev2 = false;
    private boolean yPrev1 = false;
    private boolean yPrev2 = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");

        turretXZ = hardwareMap.get(DcMotorEx.class, "turretXZ");
        turretYZ = new SimpleServo(hardwareMap, "turretYZ", 0, 180, AngleUnit.DEGREES);

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        pusher = new SimpleServo(hardwareMap, "pusher", 0, 180, AngleUnit.DEGREES);
        pusher.turnToAngle(RETRACT_ANGLE, AngleUnit.DEGREES);

        imu = hardwareMap.get(BHI260IMU.class, "imu");

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

//        driveBL.setDirection(DcMotor.Direction.REVERSE); // was set
//        driveBR.setDirection(DcMotor.Direction.REVERSE);
//        driveFL.setDirection(DcMotor.Direction.REVERSE); // was set
//        driveFR.setDirection(DcMotor.Direction.REVERSE);

        turretXZ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretXZ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        mecanum = new MecanumDrive(true, driveFL, driveFR, driveBL, driveBR);

        turretYZAngle = 90.0;
        turretYZ.turnToAngle(turretYZAngle, AngleUnit.DEGREES);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
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

        double rotationPower = (Math.abs(rx) > STICK_DEADZONE) ? rx * MAX_ROTATION_POWER : 0.0;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        mecanum.setMaxSpeed(driveScale);
        mecanum.driveFieldCentric(lx, ly, rotationPower, heading);

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
            // TODO: Implement shoot sequence here (start flywheel, run pusher sequence, etc.)
        }
        aPrev1 = aNow1;
        aPrev2 = aNow2;

        boolean yNow1 = gamepad1.y;
        boolean yNow2 = gamepad2.y;
        boolean yPressed = (yNow1 && !yPrev1) || (yNow2 && !yPrev2);
        if (yPressed) {
            // TODO: Implement auto-aim routine here (use vision / imu / encoders)
        }
        yPrev1 = yNow1;
        yPrev2 = yNow2;

        // flywheelMotor.setPower(0.0);

        // telemetry
        telemetry.addData("L stick", "lx=%.2f ly=%.2f", lx, ly);
        telemetry.addData("R stick rx", "%.2f", rx);
        telemetry.addData("Drive scale (rt)", "%.2f (rt=%.2f)", driveScale, rt);
        telemetry.addData("Rotation Power", "%.3f", rotationPower);
        telemetry.addData("Turret XZ Power", "%.2f", turretCmdXZ);
        telemetry.addData("Turret YZ Angle", "%.1f", turretYZAngle);
        telemetry.addData("Intake Power", "%.2f", intakeCmd);
        telemetry.addData("Flywheel Power", "%.2f", flywheelMotor.getPower());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.update();
    }

    @Override
    public void stop() {
        mecanum.stop();
        turretXZ.setPower(0.0);
        intakeMotor.setPower(0.0);
        flywheelMotor.setPower(0.0);
        pusher.turnToAngle(RETRACT_ANGLE, AngleUnit.DEGREES);
    }
}