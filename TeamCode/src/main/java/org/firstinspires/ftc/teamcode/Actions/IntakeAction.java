package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Intake motor controller with on/off/reverse/toggle and reverse-on-stop behavior.
 */
public class IntakeAction implements RobotAction {
    private final DcMotorEx motor;
    private final double intakePower;
    private double currentPower = 0.0;

    // Reverse-on-stop parameters
    private final double reversePower = -0.2;
    private final long reverseDurationMs = 2000; // 2 seconds
    private long reverseStartTime = 0;
    private boolean reversing = false;

    public IntakeAction(DcMotorEx motor, double intakePower) {
        this.motor = motor;
        this.intakePower = Math.abs(intakePower);
        motor.setPower(0.0);
    }

    public void on() {
        reversing = false;
        setPower(intakePower);
    }

    public void reverse() {
        reversing = false;
        setPower(-intakePower);
    }

    public void off() {
        // start reverse-on-stop sequence
        reversing = true;
        reverseStartTime = System.currentTimeMillis();
        setPower(reversePower);
    }

    public void off(boolean reverse) {
        reversing = false;
        setPower(0.0);
    }

    public void toggle() {
        if (Math.abs(currentPower) < 1e-6 && !reversing) {
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
        // handle reverse-on-stop timing
        if (reversing) {
            long elapsed = System.currentTimeMillis() - reverseStartTime;
            if (elapsed >= reverseDurationMs) {
                reversing = false;
                setPower(0.0); // fully stop after 2 seconds
            }
        }
    }

    @Override
    public void stop() {
        off();
    }

    @Override
    public boolean isBusy() {
        return Math.abs(currentPower) > 1e-6 || reversing;
    }
}