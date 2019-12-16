package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;

@Autonomous

public class AutoMech extends LinearOpMode {
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
        
        
    }
}
