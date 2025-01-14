package org.firstinspires.ftc.teamcode.Autonomous;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "SlidesAuto", group = "Autonomous")
public class SlidesAuto extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift; //lift is left
        private DcMotorEx lift2; //lift2 is right




        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");
            lift2 = hardwareMap.get(DcMotorEx.class,"slideMotorRight");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        }




        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    lift2.setPower(0.8);

                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    lift2.setPower(0);
                    return false;
                }



            }
        }


        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    lift2.setPower(-0.8);

                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }





    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "clawServo");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.0);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.45);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }


    public class ArmServo {
        private Servo armServo;

        public ArmServo(HardwareMap hardwareMap) {
            armServo = hardwareMap.get(Servo.class,"clawServo");
        }

        public class RaiseArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(.772);   //TO CHANGE
                return false;
            }
        }

        public Action raiseArm() {
            return new RaiseArm();
        }

        public class LowerArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(.21);
                return false;
            }
        }

        public Action lowerArm() {
            return new LowerArm();
        }


    }


    public class mainHanging {

        public void hangSpecimen(HardwareMap hwMap) {
                Lift slidePart = new Lift(hwMap);
                Actions.runBlocking(slidePart.liftUp());

                ArmServo armPart = new ArmServo(hwMap);
                Actions.runBlocking(armPart.lowerArm());

                Actions.runBlocking(slidePart.liftDown());

                Claw clawPart = new Claw(hwMap);
                Actions.runBlocking(clawPart.openClaw());
        }
    }







    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-9.73, 35.77), Math.toRadians(270))
                .waitSeconds(5.0);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-35, 36.16), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-35, 12), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-41, 12), Math.toRadians(90))
                // Move up
                .splineToConstantHeading(new Vector2d(-41, 50), Math.toRadians(90))
                // Move right
                .splineToConstantHeading(new Vector2d(-45, 50), Math.toRadians(270))
                // Move down
                .splineToConstantHeading(new Vector2d(-45, 12), Math.toRadians(270))
                // Move right
                .splineToConstantHeading(new Vector2d(-51, 12), Math.toRadians(90))
                // Move up
                .splineToConstantHeading(new Vector2d(-50, 50), Math.toRadians(90))
                // Move right
                .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(270))
                // Move down
                .splineToConstantHeading(new Vector2d(-53, 12), Math.toRadians(270))
                // Move right
                .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(90))
                // Move up
                .splineToConstantHeading(new Vector2d(-60, 50), Math.toRadians(90))
                // Move back
                .splineToConstantHeading(new Vector2d(-55, 47), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-42,60), Math.toRadians(180));
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .build();


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());



        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        mainHanging hanger = new mainHanging();


        Lift slidePart = new Lift(hardwareMap);

        ArmServo armPart = new ArmServo(hardwareMap);


        Claw clawPart = new Claw(hardwareMap);


        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                        slidePart.liftUp(),
                        armPart.lowerArm(),
                        slidePart.liftDown(),
                        new SleepAction(0.5),//waits 0.5 seconds
                        clawPart.openClaw(),
                        tab2.build(),
                        trajectoryActionCloseOut
                )
        );
    }
}