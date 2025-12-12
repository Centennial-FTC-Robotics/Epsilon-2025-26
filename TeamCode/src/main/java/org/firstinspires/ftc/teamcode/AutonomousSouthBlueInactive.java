package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutonomousSouthBlueInactive")
public class AutonomousSouthBlueInactive extends LinearOpMode {

    private DcMotorEx driveFL;
    private DcMotorEx driveFR;
    private DcMotorEx driveBL;
    private DcMotorEx driveBR;

    @Override
    public void runOpMode() throws InterruptedException {
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");
        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");

        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFL.setDirection(DcMotorEx.Direction.REVERSE);
        driveBL.setDirection(DcMotorEx.Direction.REVERSE);
        driveFR.setDirection(DcMotorEx.Direction.FORWARD);
        driveBR.setDirection(DcMotorEx.Direction.FORWARD);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            double power = 0.5;
            double strafeTime = 2.0; // seconds
            ElapsedTime timer = new ElapsedTime();
            timer.reset();

            while (opModeIsActive()) {
                if (timer.seconds() < strafeTime) {
                    driveFL.setPower(-power);
                    driveFR.setPower( power);
                    driveBL.setPower( power);
                    driveBR.setPower(-power);

                    telemetry.addData("Action", "Strafing");
                    telemetry.addData("Time", "%.2f / %.2f", timer.seconds(), strafeTime);
                } else {
                    driveFL.setPower(0);
                    driveFR.setPower(0);
                    driveBL.setPower(0);
                    driveBR.setPower(0);

                    telemetry.addData("Action", "Stopped");
                }

                telemetry.update();
            }
        }
    }
}