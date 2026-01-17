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

    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;

    private ButtonReader aReader1;
    private ButtonReader aReader2;
    private ButtonReader yReader1;
    private ButtonReader yReader2;
    private ButtonReader bReader1;
    private ButtonReader bReader2;

    private ButtonReader dpadLeft1;
    private ButtonReader dpadLeft2;
    private ButtonReader dpadRight1;
    private ButtonReader dpadRight2;
    private ButtonReader dpadUp1;
    private ButtonReader dpadUp2;
    private ButtonReader dpadDown1;
    private ButtonReader dpadDown2;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        readValues();

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
        turretXZ.setPower(turretCmdXZ);

        boolean dpadUp = dpadUp1.isDown() || dpadUp2.isDown();
        boolean dpadDown = dpadDown1.isDown() || dpadDown2.isDown();
        if (dpadUp && !dpadDown) {
            turretYZAngle += TURRET_YZ_STEP;
            if (turretYZAngle > TURRET_YZ_MAX) turretYZAngle = TURRET_YZ_MAX;
            turretYZ.turnToAngle(turretYZAngle, AngleUnit.DEGREES);
        } else if (dpadDown && !dpadUp) {
            turretYZAngle -= TURRET_YZ_STEP;
            if (turretYZAngle < TURRET_YZ_MIN) turretYZAngle = TURRET_YZ_MIN;
            turretYZ.turnToAngle(turretYZAngle, AngleUnit.DEGREES);
        }

        boolean intakeOn = bReader1.isDown() || bReader2.isDown();
        double intakeCmd = intakeOn ? INTAKE_POWER : 0.0;
        intakeMotor.setPower(intakeCmd);

        boolean aPressed = aReader1.wasJustPressed() || aReader2.wasJustPressed();
        if (aPressed) {
            // TODO: Implement shoot sequence here (start flywheel, run pusher sequence, etc.)
            telemetry.addData("Event", "A pressed (shoot) detected");
        }

        boolean yPressed = yReader1.wasJustPressed() || yReader2.wasJustPressed();
        if (yPressed) {
            // TODO: Implement auto-aim routine here (use vision / imu / encoders)
            telemetry.addData("Event", "Y pressed (auto-aim) detected");
        }

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