package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;




public class Slides {

    private DcMotorEx slideMotorR;
    private DcMotorEx slideMotorL;

    public Slides(final HardwareMap hMap) {
        slideMotorR = hMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorL = hMap.get(DcMotorEx.class, "slideMotorLeft");

        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public void slideUp(int targetPosition, double power) {
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorL.setTargetPosition(targetPosition); //
        slideMotorR.setTargetPosition(targetPosition);

        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotorL.setPower(power); //
        slideMotorR.setPower(power); //

        while (slideMotorL.isBusy()) {}

        slideMotorL.setPower(0.0);
        slideMotorR.setPower(0.0);
    }



}
