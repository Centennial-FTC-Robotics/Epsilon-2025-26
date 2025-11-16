 package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorSubsystem;

@TeleOp(name="Decode Color Sensor Test")
public class TeleOpColorSensorTest extends LinearOpMode {
    private ColorSensorSubsystem colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = new ColorSensorSubsystem(hardwareMap, "color_sensor");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("R", colorSensor.getRed());
            telemetry.addData("G", colorSensor.getGreen());
            telemetry.addData("B", colorSensor.getBlue());
            telemetry.addData("Alpha", colorSensor.getAlpha());

            telemetry.addData("Detected", colorSensor.getDetectedColor());
            telemetry.update();
        }
    }
}
