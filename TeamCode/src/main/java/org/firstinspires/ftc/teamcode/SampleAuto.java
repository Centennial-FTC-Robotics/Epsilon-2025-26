package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Autonomous(name = "SampleAuto")
public class SampleAuto extends LinearOpMode {


    private DcMotorEx slideMotorR;
    private DcMotorEx slideMotorL;


    @Override
    public void runOpMode() throws InterruptedException {


        slideMotorR = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorL = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");

        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        waitForStart();
        drivetrain.move(24.0,"strafe");

        drivetrain.move(12.0, "drive");


        slideMotorR.setPower(1.0);
        slideMotorL.setPower(1.0);


        drivetrain.move(12.0, "drive");


        slideMotorR.setPower(-1.0);
        slideMotorL.setPower(-1.0);


        //hook





        //raise slides
        //another move 12 to specimens
        //lower slides to hook speciemn




    }
}
