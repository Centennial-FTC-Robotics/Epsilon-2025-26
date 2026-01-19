package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Non-blocking servo pusher sequence: extend -> hold -> retract.
 * Call update() each loop.
 */
public class PusherAction implements RobotAction {
    public enum State { IDLE, EXTENDING, HOLDING, RETRACTING }

    private final ServoEx servo;
    private final double retractAngle;
    private final double pushAngle;
    private final double extendTime; // seconds to wait for extension (optional)
    private final double holdTime;   // seconds to hold while extended

    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.IDLE;

    public PusherAction(ServoEx servo,
                        double retractAngleDeg,
                        double pushAngleDeg,
                        double extendTimeSeconds,
                        double holdTimeSeconds) {
        this.servo = servo;
        this.retractAngle = retractAngleDeg;
        this.pushAngle = pushAngleDeg;
        this.extendTime = extendTimeSeconds;
        this.holdTime = holdTimeSeconds;

        // ensure starts retracted
        servo.turnToAngle(retractAngle, AngleUnit.DEGREES);
    }

    /**
     * Starts a single push sequence (non-blocking).
     */
    public void startPush() {
        if (state != State.IDLE) return;
        servo.turnToAngle(pushAngle, AngleUnit.DEGREES);
        timer.reset();
        state = State.EXTENDING;
    }

    @Override
    public void update() {
        switch (state) {
            case IDLE:
                // nothing
                break;
            case EXTENDING:
                if (timer.seconds() >= extendTime) {
                    timer.reset();
                    state = State.HOLDING;
                }
                break;
            case HOLDING:
                if (timer.seconds() >= holdTime) {
                    servo.turnToAngle(retractAngle, AngleUnit.DEGREES);
                    timer.reset();
                    state = State.RETRACTING;
                }
                break;
            case RETRACTING:
                // give servo time to retract then go idle
                if (timer.seconds() >= extendTime) {
                    state = State.IDLE;
                }
                break;
        }
    }

    @Override
    public void stop() {
        servo.turnToAngle(retractAngle, AngleUnit.DEGREES);
        state = State.IDLE;
    }

    @Override
    public boolean isBusy() {
        return state != State.IDLE;
    }
}
