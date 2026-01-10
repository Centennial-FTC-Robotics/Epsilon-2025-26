package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class FlywheelTuningTutorial extends OpMode {

    public DcMotorEx flywheelMotor;
    public double highVelocity = 1500;
    public double lowVelocity = 900;
    public double targetVelocity = highVelocity;

    double F = 0;
    double P = 0;
    double[] stepsizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;

    @Override
    public void init(){
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        PIDFCoefficients
    }

    @Override
    public void loop(){

    }


}

