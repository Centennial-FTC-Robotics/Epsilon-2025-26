package org.firstinspires.ftc.teamcode.Actions;

/**
 * Base interface for non-blocking actions. Call update() every loop.
 */
public interface RobotAction {
    /**
     * Called every loop to progress the action state machine.
     */
    void update();

    /**
     * Stop the action and reset to idle.
     */
    void stop();

    /**
     * Returns true if the action is currently doing work (not idle).
     */
    boolean isBusy();
}