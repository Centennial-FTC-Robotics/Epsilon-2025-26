package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Drivetrain {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    private boolean slowMode = false;

    public Slides slidesPart;

    public Drivetrain(final HardwareMap hMap) {
        frontLeft = hMap.get(DcMotor.class, "frontLeft");
        frontRight = hMap.get(DcMotor.class, "frontRight");
        backLeft = hMap.get(DcMotor.class, "backLeft");
        backRight = hMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //slides

        Slides slidePart = new Slides(hMap);


    }

    private double inches_to_ticks(double inches) {
        double wheel_diameter = 95 / 25.4;
        double circumference = wheel_diameter * Math.PI;
        double revs = inches / circumference;
        return revs * 537.6;
    }

    public void move(double inches, String mode) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        int position = (int) inches_to_ticks(inches);
        if (mode == "drive") {
            frontLeft.setTargetPosition(-position);
            frontRight.setTargetPosition(-position);
            backLeft.setTargetPosition(-position);
            backRight.setTargetPosition(-position);
        } else if (mode == "strafe") {
            frontLeft.setTargetPosition(-position);
            frontRight.setTargetPosition(position);
            backLeft.setTargetPosition(position);
            backRight.setTargetPosition(-position);
        } else if (mode == "rotate") {
            frontLeft.setTargetPosition(-position);
            frontRight.setTargetPosition(position);
            backLeft.setTargetPosition(-position);
            backRight.setTargetPosition(position);
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();


        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("frontLeft: ", frontLeft.getCurrentPosition());
            packet.put("frontRight: ", frontRight.getCurrentPosition());
            packet.put("backLeft: ", backLeft.getCurrentPosition());
            packet.put("backRight: ", backRight.getCurrentPosition());
            packet.put("status", "alive");
            dashboard.sendTelemetryPacket(packet);
        }

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);


        //drive :)
    }




    public void hangSpecimen() {

        slidesPart.incrementSlidePos(5);

    }


}
