package org.firstinspires.ftc.teamcode.tuning;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Calculate FeedForward")
public class CalculateFeedForward extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        // Run motor at full power
        flywheel.setPower(1.0);

        // Allow flywheel to reach steady-state
        sleep(2000);

        // Measure velocity (ticks/sec)
        double maxTicksPerSec = flywheel.getVelocity();

        // Calculate kF
        double kF = 32767.0 / maxTicksPerSec;

        telemetry.addData("Max ticks/sec", maxTicksPerSec);
        telemetry.addData("Calculated kF", kF);
        telemetry.update();

        // Keep OpMode alive
        while (opModeIsActive()) {
            idle();
        }
    }
}
