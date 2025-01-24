package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Autonomous(name = "encoderAuto")
public class encoderSampleAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        waitForStart();


        drivetrain.move(24.0,"strafe");

        drivetrain.move(-24.0,"drive");

        drivetrain.move(10.0,"rotate");

        drivetrain.move(15.0,"drive");

        //place in

        drivetrain.move(-10.0,"rotate");

        drivetrain.move(48.0,"drive");

        drivetrain.move(-24.0,"strafe");



    }






}
