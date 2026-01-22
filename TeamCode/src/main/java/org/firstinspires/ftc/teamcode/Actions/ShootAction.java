package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Coordinates Flywheel + Pusher + Intake for a single-shot sequence:
 *
 * 1) Spin up flywheel
 * 2) Wait for flywheel ready
 * 3) Extend pusher
 * 4) Start intake
 * 5) Wait ~3s
 * 6) Stop flywheel + intake
 * 7) Retract pusher
 *
 * Non-blocking: call update() every loop.
 */
public class ShootAction implements RobotAction {

    public enum State {
        IDLE,
        SPINNING_UP,
        PUSHING,
        SHOOTING,
        RETRACTING
    }

    private final FlywheelAction flywheel;
    private final PusherAction pusher;
    private final IntakeAction intake;

    private final double shootTimeSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    private State state = State.IDLE;

    public ShootAction(FlywheelAction flywheel,
                       PusherAction pusher,
                       IntakeAction intake,
                       double shootTimeSeconds) {
        this.flywheel = flywheel;
        this.pusher = pusher;
        this.intake = intake;
        this.shootTimeSeconds = shootTimeSeconds;
    }

    /** Start the full shooting sequence. */
    public void start() {
        if (state != State.IDLE) return;

        flywheel.spinUp();
        timer.reset();
        state = State.SPINNING_UP;
    }

    @Override
    public void update() {
        switch (state) {

            case IDLE:
                break;

            case SPINNING_UP:
                if (flywheel.isAtSpeed()) {
                    pusher.startPush();
                    state = State.PUSHING;
                }
                break;

            case PUSHING:
                if (!pusher.isBusy()) {
                    intake.on();
                    timer.reset();
                    state = State.SHOOTING;
                }
                break;

            case SHOOTING:
                if (timer.seconds() >= shootTimeSeconds) {
                    intake.off();
                    flywheel.stopFlywheel();
                    pusher.stop(); // retract
                    state = State.RETRACTING;
                }
                break;

            case RETRACTING:
                if (!pusher.isBusy()) {
                    state = State.IDLE;
                }
                break;
        }
    }

    @Override
    public void stop() {
        flywheel.stopFlywheel();
        intake.off();
        pusher.stop();
        state = State.IDLE;
    }

    @Override
    public boolean isBusy() {
        return state != State.IDLE;
    }
}