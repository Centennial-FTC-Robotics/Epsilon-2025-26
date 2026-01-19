package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Simple intake motor controller with on/off/reverse/toggle.
 */
public class IntakeAction implements RobotAction {
    private final DcMotorEx motor;
    private final double intakePower;
    private double currentPower = 0.0;

    public IntakeAction(DcMotorEx motor, double intakePower) {
        this.motor = motor;
        this.intakePower = Math.abs(intakePower);
        motor.setPower(0.0);
    }

    public void on() {
        setPower(intakePower);
    }

    public void reverse() {
        setPower(-intakePower);
    }

    public void off() {
        setPower(0.0);
    }

    public void toggle() {
        if (Math.abs(currentPower) < 1e-6) {
            on();
        } else {
            off();
        }
    }

    public void setPower(double power) {
        currentPower = power;
        motor.setPower(power);
    }

    public double getPower() {
        return currentPower;
    }

    @Override
    public void update() {
        // nothing to do per-loop for this simple driver
    }

    @Override
    public void stop() {
        off();
    }

    @Override
    public boolean isBusy() {
        return Math.abs(currentPower) > 1e-6;
    }
}
