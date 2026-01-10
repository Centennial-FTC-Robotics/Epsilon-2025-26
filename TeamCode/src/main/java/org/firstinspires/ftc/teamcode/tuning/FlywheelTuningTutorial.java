package org.firstinspires.ftc.teamcode.tuning;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class FlywheelTuningTutorial extends OpMode {

    public DcMotorEx flywheelMotor;
    public double highVelocity = 1500;
    public double lowVelocity = 900;
    public double targetVelocity = highVelocity;

    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;


    @Override
    public void init(){
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Initialization Successful");
        telemetry.update();
    }

    @Override
    public void loop(){
        //gamepad commands, get velocity, update telemetry
        if (gamepad1.bWasPressed()){
            if(targetVelocity == highVelocity){
                targetVelocity = lowVelocity;
            }
            else{targetVelocity = highVelocity;}
        }

        if(gamepad1.b){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpad_up){
            P += stepSizes[stepIndex];
        }

        if(gamepad1.dpad_down){
            P -= stepSizes[stepIndex];
        }



    }


}

