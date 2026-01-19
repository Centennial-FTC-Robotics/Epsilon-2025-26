package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Coordinates FlywheelAction + PusherAction to perform a single-shot sequence:
 * 1) Spin flywheel (if not already)
 * 2) Wait for spin-up
 * 3) Start pusher push
 * 4) Wait for pusher to finish then optionally stop flywheel
 *
 * Non-blocking: call update() each loop.
 */
public class ShootAction implements RobotAction {
    public enum State { IDLE, SPINNING_UP, PUSHING, WAITING_AFTER_PUSH, COMPLETE }

    private final FlywheelAction flywheel;
    private final PusherAction pusher;
    private final boolean keepFlywheelAfterShot;
    private final ElapsedTime timer = new ElapsedTime();

    private State state = State.IDLE;

    public ShootAction(FlywheelAction flywheel, PusherAction pusher, boolean keepFlywheelAfterShot) {
        this.flywheel = flywheel;
        this.pusher = pusher;
        this.keepFlywheelAfterShot = keepFlywheelAfterShot;
    }

    /** Starts the coordinated shoot sequence. */
    public void startSingleShot() {
        if (state != State.IDLE) return;
        flywheel.spinUp(); // uses flywheel's configured power
        timer.reset();
        state = State.SPINNING_UP;
    }

    @Override
    public void update() {
        switch (state) {
            case IDLE:
                // nothing
                break;
            case SPINNING_UP:
                // wait until flywheel reports at-speed (or some timeout)
                if (flywheel.isAtSpeed()) {
                    // start pusher
                    pusher.startPush();
                    state = State.PUSHING;
                }
                break;
            case PUSHING:
                // wait for pusher to finish
                if (!pusher.isBusy()) {
                    timer.reset();
                    state = State.WAITING_AFTER_PUSH;
                }
                break;
            case WAITING_AFTER_PUSH:
                // short delay to ensure the projectile clears
                if (timer.seconds() >= 1) {
                    if (!keepFlywheelAfterShot) {
                        flywheel.stopFlywheel();
                    }
                    state = State.COMPLETE;
                }
                break;
            case COMPLETE:
                // mark idle after a tick so callers can re-trigger
                state = State.IDLE;
                break;
        }
    }

    @Override
    public void stop() {
        flywheel.stopFlywheel();
        pusher.stop();
        state = State.IDLE;
    }

    @Override
    public boolean isBusy() {
        return state != State.IDLE;
    }
}
