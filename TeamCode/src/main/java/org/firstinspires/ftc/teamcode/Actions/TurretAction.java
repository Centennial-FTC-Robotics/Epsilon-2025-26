package org.firstinspires.ftc.teamcode.Actions;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Simple turret controller:
 * - rotateXZ(power) will set turret XZ motor power (-1..1)
 * - setYZAngle(angle) moves the YZ servo to a target angle
 * - stepYZ(up/down) convenience methods
 */
public class TurretAction implements RobotAction {

    private final DcMotorEx turretXZ;
    private final ServoEx turretYZ;

    private final double yzMin;
    private final double yzMax;
    private final double yzStep;

    private double yzAngleTarget;
    private boolean busy = false;

    public TurretAction(DcMotorEx turretXZ, ServoEx turretYZ,
                        double yzMinDeg, double yzMaxDeg, double yzStepDeg,
                        double initialYZAngleDeg) {
        this.turretXZ = turretXZ;
        this.turretYZ = turretYZ;
        this.yzMin = yzMinDeg;
        this.yzMax = yzMaxDeg;
        this.yzStep = yzStepDeg;

        this.yzAngleTarget = Math.max(yzMin, Math.min(yzMax, initialYZAngleDeg));
        turretYZ.turnToAngle(this.yzAngleTarget, AngleUnit.DEGREES);
    }

    /** Set XZ motor power (-1..1). Call this every loop or call stopXZ() to stop. */
    public void rotateXZ(double power) {
        turretXZ.setPower(power);
    }

    /** Convenience: stop rotation. */
    public void stopXZ() {
        turretXZ.setPower(0.0);
    }

    /** Set YZ target angle (clamped) and immediately command servo to move. */
    public void setYZAngle(double angleDeg) {
        yzAngleTarget = Math.max(yzMin, Math.min(yzMax, angleDeg));
        turretYZ.turnToAngle(yzAngleTarget, AngleUnit.DEGREES);
    }

    public void stepYZUp() {
        setYZAngle(yzAngleTarget + yzStep);
    }

    public void stepYZDown() {
        setYZAngle(yzAngleTarget - yzStep);
    }

    public double getYZAngle() {
        return yzAngleTarget;
    }

    @Override
    public void update() {
        // turret is mostly immediate commands — no internal time-based state
        busy = false;
    }

    @Override
    public void stop() {
        stopXZ();
        // keep current YZ angle
        busy = false;
    }

    @Override
    public boolean isBusy() {
        return busy;
    }
}
