package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "ServoDiagnostic", group = "Diagnostics")
public class ServoDiagnostic extends LinearOpMode {

    private Servo rawServo;
    private SimpleServo simpleServo; // optional (FTCLib). May be null if creation fails.

    // user-tuneable increment for D-pad position adjustments
    private final double POS_STEP = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing ServoDiagnostic...");
        telemetry.update();

        // Replace "pusher" with your servo name in the robot configuration if different
        rawServo = hardwareMap.get(Servo.class, "pusher");

        // Try to create a SimpleServo for angle-based testing; it's optional.
        try {
            simpleServo = new SimpleServo(hardwareMap, "pusher", 0, 180, AngleUnit.DEGREES);
        } catch (Exception e) {
            simpleServo = null;
        }

        // show instructions while waiting for start
        while (!isStarted() && !isStopRequested()) {
            telemetry.clearAll();
            telemetry.addLine("SERVO DIAGNOSTIC (pre-start)");
            telemetry.addLine("");
            telemetry.addLine("Buttons:");
            telemetry.addLine("A -> setPosition(0.0)   X -> setPosition(1.0)");
            telemetry.addLine("B -> setPosition(0.5)   Y -> toggle direction");
            telemetry.addLine("D-pad UP/DOWN -> increment/decrement position");
            telemetry.addLine("Left Bumper -> set 0.0   Right Bumper -> set 1.0");
            telemetry.addLine("Start -> Center to 0.5 (quick CR check)");
            telemetry.addLine("");
            telemetry.addData("Raw pos (last known)", "%.3f", rawServo.getPosition());
            if (simpleServo != null) {
                telemetry.addData("SimpleServo angle (deg)", "%.1f", simpleServo.getAngle());
            } else {
                telemetry.addLine("SimpleServo: not available (FTCLib not present?)");
            }
            telemetry.update();
            idle();
        }

        waitForStart();

        boolean yPrev = false;
        boolean aPrev = false;
        boolean bPrev = false;
        boolean xPrev = false;
        boolean lbPrev = false;
        boolean rbPrev = false;
        boolean startPrev = false;
        boolean dpadUpPrev = false;
        boolean dpadDownPrev = false;

        // main control loop
        while (opModeIsActive() && !isStopRequested()) {
            boolean aNow = gamepad1.a;
            boolean bNow = gamepad1.b;
            boolean xNow = gamepad1.x;
            boolean yNow = gamepad1.y;
            boolean lbNow = gamepad1.left_bumper;
            boolean rbNow = gamepad1.right_bumper;
            boolean startNow = gamepad1.start;
            boolean dpadUpNow = gamepad1.dpad_up;
            boolean dpadDownNow = gamepad1.dpad_down;

            // A B X buttons: absolute set positions
            if (aNow && !aPrev) {
                rawServo.setPosition(0.0); // endpoint 0
            }
            if (bNow && !bPrev) {
                rawServo.setPosition(0.5); // midpoint
            }
            if (xNow && !xPrev) {
                rawServo.setPosition(1.0); // endpoint 1
            }

            // left/right bumpers quick endpoints
            if (lbNow && !lbPrev) {
                rawServo.setPosition(0.0);
            }
            if (rbNow && !rbPrev) {
                rawServo.setPosition(1.0);
            }

            // Start button: set to center (0.5) — useful quick check for CR behavior
            if (startNow && !startPrev) {
                rawServo.setPosition(0.5);
            }

            // D-pad incremental adjustments
            if (dpadUpNow && !dpadUpPrev) {
                double next = clamp(rawServo.getPosition() + POS_STEP, 0.0, 1.0);
                rawServo.setPosition(next);
            }
            if (dpadDownNow && !dpadDownPrev) {
                double next = clamp(rawServo.getPosition() - POS_STEP, 0.0, 1.0);
                rawServo.setPosition(next);
            }

            // Y toggles direction
            if (yNow && !yPrev) {
                Servo.Direction cur = rawServo.getDirection();
                Servo.Direction nxt = (cur == Servo.Direction.FORWARD) ? Servo.Direction.REVERSE : Servo.Direction.FORWARD;
                rawServo.setDirection(nxt);
            }

            // Show telemetry to observe values and behavior
            telemetry.clearAll();
            telemetry.addLine("SERVO DIAGNOSTIC (running)");
            telemetry.addLine("");
            telemetry.addLine("Controls:");
            telemetry.addLine("A: 0.0  B: 0.5  X: 1.0");
            telemetry.addLine("D-pad up/down: +/- " + POS_STEP);
            telemetry.addLine("Y: toggle direction");
            telemetry.addLine("LB/RB: set endpoints  Start: center 0.5");
            telemetry.addLine("");

            telemetry.addData("Raw.getPosition()", "%.3f", rawServo.getPosition());
            telemetry.addData("Raw.getDirection()", rawServo.getDirection().toString());
            if (simpleServo != null) {
                // report SimpleServo's angle (degrees)
                double ang;
                try {
                    ang = simpleServo.getAngle();
                    telemetry.addData("SimpleServo.getAngle()", "%.1f°", ang);
                } catch (Exception e) {
                    telemetry.addData("SimpleServo.getAngle()", "error: %s", e.toString());
                }
            } else {
                telemetry.addLine("SimpleServo: not created");
            }

            telemetry.addLine("");
            telemetry.addLine("Observations (manual):");
            telemetry.addLine("- If servo spins continuously when setPosition is used -> CONTINUOUS (CR) servo or wrong config.");
            telemetry.addLine("- If setPosition changes but physical sweep is tiny -> mechanical stop or mapping problem.");
            telemetry.update();

            // store previous states for edge detection
            aPrev = aNow;
            bPrev = bNow;
            xPrev = xNow;
            yPrev = yNow;
            lbPrev = lbNow;
            rbPrev = rbNow;
            startPrev = startNow;
            dpadUpPrev = dpadUpNow;
            dpadDownPrev = dpadDownNow;

            idle();
        }
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
