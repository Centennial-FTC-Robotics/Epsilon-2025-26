package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@Autonomous(name = "encoderAuto")
public class encoderAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(this);

        waitForStart();

        drivetrain.autoInit();

        drivetrain.move(15.0,"strafe");

        drivetrain.move(-27, "drive");

        //hang yo shi

        drivetrain.hangSpecimen();



        drivetrain.move(5.0,"drive");

        drivetrain.move(-38.0,"strafe");

        drivetrain.move(-25.0,"drive");



        //push blocks

        drivetrain.move(-11.0,"strafe");

        drivetrain.move(42.0,"drive");

        drivetrain.move(-42.0,"drive");

        drivetrain.move(-13.5,"strafe");

        drivetrain.move(42.0,"drive");

        drivetrain.move(-42.0,"drive");

        drivetrain.move(-8.5,"strafe");

        drivetrain.move(45.0,"drive");





    }






}
