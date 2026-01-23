package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Coordinates Flywheel + Intake for a single-shot sequence:
 *
 * 1) Spin up flywheel
 * 2) Wait for flywheel ready
 * 3) Start intake to feed all balls
 * 4) Wait ~shootTimeSeconds
 * 5) Stop flywheel + intake
 *
 * Non-blocking: call update() every loop.
 */
public class ShootAction implements RobotAction {

    public enum State {
        IDLE,
        SPINNING_UP,
        SHOOTING
    }

    private final FlywheelAction flywheel;
    private final IntakeAction intake;

    private final double shootTimeSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    private State state = State.IDLE;

    public ShootAction(FlywheelAction flywheel,
                       IntakeAction intake,
                       double shootTimeSeconds) {
        this.flywheel = flywheel;
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
                    intake.on();
                    timer.reset();
                    state = State.SHOOTING;
                }
                break;

            case SHOOTING:
                if (timer.seconds() >= shootTimeSeconds) {
                    intake.off();
                    flywheel.stopFlywheel();
                    state = State.IDLE;
                }
                break;
        }
    }

    @Override
    public void stop() {
        flywheel.stopFlywheel();
        intake.off();
        state = State.IDLE;
    }

    @Override
    public boolean isBusy() {
        return state != State.IDLE;
    }
}