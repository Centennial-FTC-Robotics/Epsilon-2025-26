package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {

    public Servo cool;
    public static double servoPos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException{
        cool = hardwareMap.get(Servo.class, "coolServo");

        waitForStart();

        while (opModeIsActive()){
            cool.setPosition(servoPos);
        }
    }
}
