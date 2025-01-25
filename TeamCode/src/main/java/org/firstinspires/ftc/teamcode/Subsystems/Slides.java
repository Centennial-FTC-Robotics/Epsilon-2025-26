package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class Slides {

    public static double slideP = 0.005;
    public static double slideI = 0.0001;
    public static double slideD = 0;
    public static double slideF = 0.1;
    public static int errorThreshold = 5;

    public int slidesTarget = 0;

    public int errorSum = 0;
    public int lastError = 0;
    public long lastTime = 0;

    public static int off = 0;
    public int[] targets = {0, 350-off, 420-off, 500-off, 580-off, 660-off, 740-off, 820-off,
            900-off, 980-off};
    public double manualPower = 0;
    public static double maxDownSpeed = 0.35;
    public int pos = -1;
    private DcMotorEx slideMotorR;
    private DcMotorEx slideMotorL;

    public Slides(LinearOpMode opmode) {
        //slideMotorR = opmode.hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorL = opmode.hardwareMap.get(DcMotorEx.class, "slideMotorLeft");

        //slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //slideMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    /*
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

     */

    public void update() {

        long t = System.currentTimeMillis();

        if(lastTime == 0) {
            lastTime = t-1;
        }


        pos = -slideMotorL.getCurrentPosition();
        double error = slidesTarget - pos;

        double speed = (double)(error-lastError)/(double)(t-lastTime);

        if(Math.abs(error) < errorThreshold) {
            errorSum = 0;
        } else {
            errorSum += error;
        }

        double power;
        if(Math.abs(manualPower) > 0.05) {
            power = Range.clip(manualPower + slideF, -1, 1);
            slidesTarget = pos;
        } else {
            power = Range.clip(error*slideP + errorSum*slideI + +speed*slideD + slideF,
                    -maxDownSpeed, 1);
        }


        slideMotorL.setPower(power);


        lastTime = t;
    }

    public void incrementSlidePos(int inc) {
        for(int i = 0; i < targets.length; i++) {
            if(targets[i] >= slidesTarget) {
                slidesTarget = targets[Range.clip((i+inc), 0, targets.length-1)];
                break;
            }
        }
    }

    public void retractSlides() {
        slidesTarget = 0;
    }

    public void setManualSlidePower(double power) {
        if(Math.abs(power) < 0.05 || (power < 0 && pos < targets[1])) {
            this.manualPower = 0;
            return;
        }
        this.manualPower = power;
    }
}
