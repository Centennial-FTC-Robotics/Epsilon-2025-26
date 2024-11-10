package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private DcMotorEx driveBL;
    private DcMotorEx driveBR;
    private DcMotorEx driveFL;
    private DcMotorEx driveFR;

    @Override
    public void runOpMode() throws InterruptedException {
        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");

        // SWAP THESE IF NEEDED
        driveBL.setDirection(DcMotorEx.Direction.REVERSE);
        //driveBR.setDirection(DcMotorEx.Direction.REVERSE);
        driveFL.setDirection(DcMotorEx.Direction.REVERSE);
        driveFR.setDirection(DcMotorEx.Direction.REVERSE);

        driveBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        driveFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        driveBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
}
