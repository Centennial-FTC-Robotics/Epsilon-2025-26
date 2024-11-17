package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
        slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // SWAP THESE IF NEEDED
        driveBL.setDirection(DcMotorEx.Direction.REVERSE);
//        driveBR.setDirection(DcMotorEx.Direction.REVERSE);
        driveFL.setDirection(DcMotorEx.Direction.REVERSE);
        driveFR.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();


        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            driveBL.setPower((y - x + rx) / d);
            driveBR.setPower((y + x - rx) / d);
            driveFL.setPower((y + x + rx) / d);
            driveFR.setPower((y - x - rx) / d);



            double slidePowerUp = gamepad1.right_trigger;
            double slidePowerDown = -gamepad1.left_trigger;

            slideMotorL.setPower(slidePowerUp > 0 ? slidePowerUp : slidePowerDown);
            slideMotorR.setPower(slidePowerUp > 0 ? slidePowerUp : slidePowerDown);


            if(gamepad1.a) {
                slideServo.setPosition(0.0);
            } else if (gamepad1.b) {
                slideServo.setPosition(0.5);
            } else if (gamepad1.y) {
                slideServo.setPosition(1.0);
            }


            if (gamepad1.left_bumper) {
                clawServo.setPosition(1.0);
            } else if (gamepad1.right_bumper) {
                clawServo.setPosition(0.0);
            }


        }





    }
}