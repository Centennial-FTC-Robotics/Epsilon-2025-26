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

        drivetrain.autoInit();

        drivetrain.move(24.0,"strafe");

        drivetrain.move(-27.0, "drive");

        //hang yo shi





        drivetrain.move(5.0,"drive");

        drivetrain.move(-38.0,"strafe");

        drivetrain.move(-25.0,"drive");



        //push blocks

        drivetrain.move(-11.0,"strafe");

        drivetrain.move(42.0,"drive");

        drivetrain.move(-42.0,"drive");

        drivetrain.move(-11.5,"strafe");

        drivetrain.move(42.0,"drive");

        drivetrain.move(-42.0,"drive");

        drivetrain.move(-7.75,"strafe");

        drivetrain.move(45.0,"drive");








    }






}
