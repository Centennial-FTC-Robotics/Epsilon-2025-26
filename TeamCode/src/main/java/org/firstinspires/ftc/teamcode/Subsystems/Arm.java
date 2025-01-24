package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private Servo armServo;

    public Arm(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(Servo.class, "armServo");
    }

    public void move(double position) {
        armServo.setPosition(position);
    }
}
