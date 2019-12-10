package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp

public class TeleMech extends LinearOpMode {
    private DcMotor angleMotor;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor liftMotor;
    private Gyroscope imu_1;
    private Gyroscope imu;
    private Servo foundationServo;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "FRONT_LEFT");
        frontRight = hardwareMap.get(DcMotor.class, "FRONT_RIGHT");
        backLeft = hardwareMap.get(DcMotor.class, "BACK_LEFT");
        backRight = hardwareMap.get(DcMotor.class, "BACK_RIGHT");
        angleMotor = hardwareMap.get(DcMotor.class, "ANGLE_MOTOR");
        liftMotor = hardwareMap.get(DcMotor.class, "LIFT_MOTOR");
        foundationServo = hardwareMap.get(Servo.class, "FOUNDATION");
        
        waitForStart();
        
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        
        boolean liftMode = false;
        boolean liftModeChanged = false;
        boolean slow = false;
        boolean slowChanged = false;
        boolean servoDown = false;
        boolean servoChanged = false;
        
        double r, theta;
        double powerFL, powerFR, powerBL, powerBR;
        double scale = 1.0;
    
        while (opModeIsActive()) {
            
            if (gamepad1.left_stick_button && !liftModeChanged) {
                liftMode = !liftMode;
                liftModeChanged = true;
            } else if (!gamepad1.left_stick_button) {
                liftModeChanged = false;
            }
            
            if (gamepad1.a && !slowChanged) {
                slow = !slow;
                slowChanged = true;
            } else if (!gamepad1.a) {
                slowChanged = false;
            }
            
            if (gamepad1.left_bumper && !servoChanged) {
                servoDown = !servoDown;
                servoChanged = true;
            } else if (!gamepad1.left_bumper) {
                servoChanged = false;
            }
            
            if (servoDown) { foundationServo.setPosition(0.5); } else { foundationServo.setPosition(0); }
            
            if (slow) { scale = 0.5; } else { scale = 1.0; }
            
            if (!liftMode) {
                r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                theta = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                
                powerFL = (r * Math.cos(theta)) - gamepad1.right_stick_x;
                powerFR = (r * Math.sin(theta)) + gamepad1.right_stick_x;
                powerBL = (r * Math.sin(theta)) - gamepad1.right_stick_x;
                powerBR = (r * Math.cos(theta)) + gamepad1.right_stick_x;
                
                frontLeft.setPower(powerFL * scale);
                frontRight.setPower(powerFR * scale);
                backLeft.setPower(powerBL * scale);
                backRight.setPower(powerBR * scale);
            } else {
                angleMotor.setPower(gamepad1.left_stick_y * scale);
                liftMotor.setPower(gamepad1.right_stick_y * scale);
            }
        }
    }
}
