package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo clawServo;

    public Claw(LinearOpMode opmode) {

        clawServo = opmode.hardwareMap.get(Servo.class, "clawServo");
    }

    public void open() {
        clawServo.setPosition(0.7);
    }

    public void close() {
        clawServo.setPosition(0.51);
    }
}


























































