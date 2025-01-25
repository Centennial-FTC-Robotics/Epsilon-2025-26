package org.firstinspires.ftc.teamcode;


import android.widget.ToggleButton;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp (name = "LibraryTeleOp")
public class LibraryTeleOp extends LinearOpMode {

    private DcMotorEx driveBL;
    private DcMotorEx driveBR;
    private DcMotorEx driveFL;
    private DcMotorEx driveFR;


    private DcMotorEx slideMotorR;
    private DcMotorEx slideMotorL;

    private Servo armServo;
    private Servo armServo2;

    private Servo clawServo;

    //private boolean clawCurrentlyClosed = true;

    public static double servoPos = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        //Slides slides = new Slides(this);
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");
        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");


       // slideMotorR = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorL = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");

        armServo = hardwareMap.get(Servo.class, "armServo"); // change in the configuration next time test
        armServo2 = hardwareMap.get(Servo.class, "armServo2");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      // slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // SWAP THESE IF NEEDED
        driveBL.setDirection(DcMotorEx.Direction.REVERSE);
        //driveBR.setDirection(DcMotorEx.Direction.REVERSE); leave commented
        //driveFL.setDirection(DcMotorEx.Direction.REVERSE); leave commented
        //driveFR.setDirection(DcMotorEx.Direction.REVERSE); leave commented

        /*
         IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);*/

        GamepadEx clawPad = new GamepadEx(gamepad2);

        ToggleButtonReader bReader = new ToggleButtonReader(
                clawPad, GamepadKeys.Button.B
        );

        ToggleButtonReader aReader = new ToggleButtonReader(
                clawPad, GamepadKeys.Button.A
        );


        ToggleButtonReader bumperReader = new ToggleButtonReader (
            clawPad, GamepadKeys.Button.LEFT_BUMPER
        );


        TriggerReader triggerReaderRight = new TriggerReader(
                clawPad, GamepadKeys.Trigger.RIGHT_TRIGGER
        );


        TriggerReader triggerReaderLeft = new TriggerReader(
                clawPad, GamepadKeys.Trigger.LEFT_TRIGGER
        );



        waitForStart();

        double slidePowerUp = 0;
        double slidePowerDown = 0;
        double armPosition = 0;

        while (opModeIsActive()) {

            bReader.readValue();

            aReader.readValue();

            bumperReader.readValue();

            triggerReaderRight.readValue();
            triggerReaderLeft.readValue();

            if (clawPad.getButton(GamepadKeys.Button.A)) {
                armPosition += 0.002;
                armPosition = Range.clip(armPosition, 0.38, 0.91);
                armServo.setPosition(armPosition);
                armServo2.setPosition(Math.abs(1-armPosition));
            } else if (clawPad.getButton(GamepadKeys.Button.B)) {
                armPosition -= 0.002;
                armPosition = Range.clip(armPosition, 0.38, 0.91);
                armServo.setPosition(armPosition);
                armServo2.setPosition(Math.abs(1-armPosition));
            }

            //clawServo.setPosition(servoPos);


            if (bumperReader.getState()) {
                clawServo.setPosition(0.7);
            } else {
                clawServo.setPosition(0.51);
            }



            /*
            if (bumperReader.getState()) {
                if (!isBumperPressed) {
                    isClawOpened = !isClawOpened;
                    isBumperPressed = true;
                    clawServo.setPosition(isClawOpened ? 0.45 : 0.0);
                }
            } else {
                isBumperPressed = false;
            }

             */

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double f = gamepad1.left_bumper ? 0.5 : 1;

            /*
            if (gamepad1.options) {
                imu.resetYaw();
            }*/

            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            /*double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;*/

            driveBL.setPower((y - x + rx) / d * f);
            driveBR.setPower((y + x - rx) / d * f);
            driveFL.setPower(((y + x + rx) / d * f) * -1.0);
            driveFR.setPower((y - x - rx) / d * f);


            slidePowerUp = gamepad1.right_trigger * 0.5;


            slidePowerDown = gamepad1.left_trigger * 0.5;



            slideMotorL.setPower((slidePowerUp - slidePowerDown));




        }
    }
}
