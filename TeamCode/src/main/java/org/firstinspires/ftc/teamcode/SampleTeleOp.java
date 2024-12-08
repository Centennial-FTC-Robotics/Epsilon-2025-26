package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp (name = "SampleTeleOp")
public class SampleTeleOp extends LinearOpMode {

    private DcMotorEx driveBL;
    private DcMotorEx driveBR;
    private DcMotorEx driveFL;
    private DcMotorEx driveFR;
    private DcMotorEx slideMotorR;
    private DcMotorEx slideMotorL;

    private Servo slideServo;

    private Servo clawServo;

    //private boolean clawCurrentlyClosed = true;


    @Override
    public void runOpMode() throws InterruptedException {

        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");

        slideMotorR = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorL = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");

        slideServo = hardwareMap.get(Servo.class, "slideServo");

        clawServo = hardwareMap.get(Servo.class, "clawServo");




        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        waitForStart();

        double slidePowerUp = 0;
        double slidePowerDown = 0;

        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

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

            driveBL.setPower((y - x + rx) / d);
            driveBR.setPower((y + x - rx) / d);
            driveFL.setPower((y + x + rx) / d);
            driveFR.setPower((y - x - rx) / d);

            /*
            driveBL.setPower(backLeftPower);
            driveBR.setPower(backRightPower);
            driveFL.setPower(frontLeftPower);
            driveFR.setPower(frontRightPower);*/

            //slidePowerUp = gamepad1.right_trigger;
            slidePowerUp = gamepad2.right_trigger * 0.8;
            //slidePowerDown = gamepad2.left_trigger;

            telemetry.addData("Current Slide Encoder Reading", slideMotorL.getCurrentPosition());
            telemetry.update();
            if (slideMotorL.getCurrentPosition() <= 10) {
                //slidePowerDown = -gamepad1.left_trigger * 0.8;
                slidePowerDown = 0;
            }
            else {
                slidePowerDown = gamepad2.left_trigger * 0.8;
            }
            slideMotorR.setPower((slidePowerUp - slidePowerDown));
           //slideMotorR.setPower(slidePowerUp > 0 ? slidePowerUp : slidePowerDown);

            if(gamepad2.a) {
                slideServo.setPosition(0.772);
            } else if (gamepad2.b) {
                slideServo.setPosition(0.65);
            } else if (gamepad2.y) {
                slideServo.setPosition(0.2);
            }

            /*
            if(gamepad1.a) {
                slideServo.setPosition(0.772);
            } else if (gamepad1.b) {
                slideServo.setPosition(0.65);
            } else if (gamepad1.y) {
                slideServo.setPosition(0.2);
            }*/



            if (gamepad2.right_bumper) {
                //closes claw
                clawServo.setPosition(0.0);
            } else if (gamepad2.left_bumper) {
                //opens claw
                clawServo.setPosition(0.45);
            }


            /*
            if (gamepad1.right_bumper) {
                //closes claw
                clawServo.setPosition(0.0);
            } else if (gamepad1.left_bumper) {
                //opens claw
                clawServo.setPosition(0.45);
            }
            */

        }
    }
}