/*

Level 1 ascent

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;


@Autonomous(name = "HangingAuto")
public class HangingAuto extends LinearOpMode {

    private DcMotorEx slideMotorR;
    private DcMotorEx slideMotorL;

    private Servo slideServo;

    private Servo clawServo;

    public void runOpMode() throws InterruptedException {

        slideMotorR = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorL = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");

        slideServo = hardwareMap.get(Servo.class, "slideServo");

        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        waitForStart();


        drivetrain.move(-21.0,"strafe");

        drivetrain.move(48.0,"drive");

        drivetrain.move(12.0, "rotate"); //tune inches for rotate

        drivetrain.move(12.0, "drive"); //subject to tuning

        Slides mainSlides = new Slides(hardwareMap);

        mainSlides.slideUp(300,0.7); //subject to change after testing

        /*
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorL.setTargetPosition(300); // 300 needs to be tuned

        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotorL.setPower(0.7); // Subject to change
        slideMotorR.setPower(0.7); // Subject to change

        while (slideMotorL.isBusy()) {}

        slideMotorL.setPower(0.0);
        slideMotorR.setPower(0.0);
        */


    }


}
