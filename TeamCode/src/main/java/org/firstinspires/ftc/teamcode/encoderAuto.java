package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Autonomous(name = "encoderAuto")
public class encoderAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        waitForStart();



        drivetrain.move(24.0,"strafe");

        drivetrain.move(-26.0, "drive");

        //hang yo shi

        drivetrain.move(5.0,"drive");

        drivetrain.move(-36.0,"strafe");

        drivetrain.move(-40.0,"drive");

        drivetrain.move(-8.0,"strafe");

        drivetrain.move(45.0,"drive");









    }






}
