package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
    BNO055IMU imu;
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
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
        
        boolean liftMode = false;
        boolean liftModeChanged = false;
        boolean slow = false;
        boolean slowChanged = false;
        boolean slow2 = false;
        boolean slow2Changed = false;
        boolean servoDown = false;
        boolean servoChanged = false;
        
        double powerFL, powerFR, powerBL, powerBR;
        double scale = 1.0;
        double scale2 = 1.0;
        
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
            
            if (gamepad2.b && !slow2Changed) {
                slow2 = !slow2;
                slow2Changed = true;
            } else if (!gamepad1.b) {
                slow2Changed = false;
            }
            
            if (gamepad1.left_bumper && !servoChanged) {
                servoDown = !servoDown;
                servoChanged = true;
            } else if (!gamepad1.left_bumper) {
                servoChanged = false;
            }
            
            
            if (servoDown) { foundationServo.setPosition(0.5); } else { foundationServo.setPosition(0); }
            
            if (slow)  { scale = 0.5; }   else { scale = 1.0; }
            if (slow2) { scale2 = 0.5; }  else { scale2 = 1.0; }
            
            if (!liftMode) {
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                
                double gyro_degrees = angles.firstAngle;
                double gyro_radians = gyro_degrees * Math.PI / 180;
                double forward = gamepad1.left_stick_y * Math.cos(gyro_radians) + gamepad1.left_stick_x * Math.sin(gyro_radians);
                double strafe = -gamepad1.left_stick_y * Math.sin(gyro_radians) + gamepad1.left_stick_x * Math.cos(gyro_radians);
                
                powerFL = forward - strafe - gamepad1.right_stick_x;
                powerFR = forward + strafe + gamepad1.right_stick_x;
                powerBL = forward + strafe - gamepad1.right_stick_x;
                powerBR = forward - strafe + gamepad1.right_stick_x;
                
                frontLeft.setPower(Range.clip(powerFL * scale, -1, 1));
                frontRight.setPower(Range.clip(powerFR * scale, -1, 1));
                backLeft.setPower(Range.clip(powerBL * scale, -1, 1));
                backRight.setPower(Range.clip(powerBR * scale, -1, 1));
                
                if (gamepad1.right_trigger > 0.0) {
                    angleMotor.setPower((double) (Range.clip(gamepad1.right_trigger, -1, 1)));
                } else if (gamepad1.left_trigger > 0.0) {
                    angleMotor.setPower(-1.0 * (double) (Range.clip(gamepad1.left_trigger, -1, 1)));
                } else {
                    angleMotor.setPower(0.0);
                }
                
                if (gamepad1.right_bumper) {
                    liftMotor.setPower(0.5);
                } else if (gamepad1.left_bumper) {
                    liftMotor.setPower(-0.5);
                } else {
                    liftMotor.setPower(0.0);
                }
                
                angleMotor.setPower(gamepad2.left_stick_y * scale2);
                liftMotor.setPower(gamepad2.right_stick_y * scale2);
            } else {
                angleMotor.setPower(gamepad1.left_stick_y * scale);
                liftMotor.setPower(gamepad1.right_stick_y * scale);
            }
        }
    }
}
