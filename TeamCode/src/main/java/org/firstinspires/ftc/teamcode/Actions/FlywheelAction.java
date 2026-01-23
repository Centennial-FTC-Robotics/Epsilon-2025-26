package org.firstinspires.ftc.teamcode.Actions;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

/**
 * Simple flywheel controller. We implement a spin-up delay to simulate waiting for
 * the flywheel to reach speed. This avoids relying on a particular encoder/velocity unit.
 *
 * If you have a well-calibrated velocity measurement, you can enhance isAtSpeed() to
 * check actual velocity against a target.
 */
public class FlywheelAction implements RobotAction {
    private final DcMotorEx motor;
    private final double rampPower; // this is the commanded power for shooting
    private final double spinUpSeconds;

    private final ElapsedTime timer = new ElapsedTime();
    private double targetPower = 0.0;
    private boolean enabled = false;

    /**
     * @param motor the DcMotorEx used for flywheel
     * @param spinUpSeconds approximate time to wait for spin-up
     * @param rampPower default shooter power (0..1) used by spinUp()
     */
    public FlywheelAction(DcMotorEx motor, double spinUpSeconds, double rampPower) {
        this.motor = motor;
        this.spinUpSeconds = spinUpSeconds;
        this.rampPower = Math.abs(rampPower);

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        motor.setPower(0.0);
    }

    /** Start spinning at the configured ramp power. */
    public void spinUp() {
        spinUp(rampPower);
    }

    /** Start spinning at specified power. */
    public void spinUp(double power) {
        targetPower = Math.signum(power) * Math.abs(power);
        motor.setPower(targetPower);
        timer.reset();
        enabled = true;
    }

    /** Stop flywheel. */
    public void stopFlywheel() {
        targetPower = 0.0;
        motor.setPower(0.0);
        enabled = false;
    }

    public void on() {
        enabled = true;
        motor.setPower(rampPower);
    }

    public void reverse() {
        enabled = true;
        motor.setPower(-rampPower);
    }

    /** Roughly indicates whether the flywheel has had time to reach operating speed. */
    public boolean isAtSpeed() {
        if (!enabled) return false;
        return timer.seconds() >= spinUpSeconds;
    }

    @Override
    public void update() {
        // no advanced control here; motor power is set when spinUp/stop called
    }

    @Override
    public void stop() {
        stopFlywheel();
    }

    @Override
    public boolean isBusy() {
        return enabled;
    }
}
