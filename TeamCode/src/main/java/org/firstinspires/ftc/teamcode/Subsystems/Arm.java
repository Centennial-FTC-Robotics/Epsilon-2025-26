package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    public Servo armServo;

    public Arm(LinearOpMode opmode) {
        armServo = opmode.hardwareMap.get(Servo.class, "armServo");
    }

    public void move(double position) {
        armServo.setPosition(position);
    }
}
