/*

This auto should be used when you are in the position just above observation zone.
If parked in the secondary location, use the ThirdParkingAuto instead

Start parallel to the observation zone with wheels touching the wall. Strafes right to the observation

 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;


@Autonomous(name = "SecondaryParkingAuto")
public class ParkingAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        waitForStart();

        drivetrain.move(30.0, "strafe");

    }





}
