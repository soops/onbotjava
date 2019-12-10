package org.firstinspires.ftc.teamcode;

import java.lang.Math;
import java.util.logging.XMLFormatter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;

@Autonomous

public class AutoMechTest extends LinearOpMode {
    
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Blinker expansion_Hub_2;
    BNO055IMU imu;
    
    public void runOpMode(){
        
        backLeft = hardwareMap.get(DcMotor.class, "BACK_LEFT");
        backRight = hardwareMap.get(DcMotor.class, "BACK_RIGHT");
        frontLeft = hardwareMap.get(DcMotor.class, "FRONT_LEFT");
        frontRight = hardwareMap.get(DcMotor.class, "FRONT_RIGHT");
        
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        
        waitForStart();
        
        move_xy(0.0, 0.0, 0.0, 2.0);
        
    }
    
    public void move (double direction, double revolutions, double rx) {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int tickPerRev = 538;
        
        double radDirection = (direction - 45) * (Math.PI / 180.0);
        
        double powerFL = 0;
        double powerFR = 0;
        double powerBR = 0;
        double powerBL = 0;
        
        double scale = 1;
        
        int targetPos = (int)(revolutions * tickPerRev);
        
        powerFL =  -Math.sin(radDirection) * scale;
        powerFR =  Math.cos(radDirection) * scale;
        powerBL =  -Math.cos(radDirection) * scale;
        powerBR =  Math.sin(radDirection) * scale;
        
        backLeft.setTargetPosition(targetPos);
        backRight.setTargetPosition(targetPos);
        frontLeft.setTargetPosition(targetPos);
        frontRight.setTargetPosition(targetPos);
        
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeft.setPower((powerFL ) - (rx / 0.7));
        frontRight.setPower((powerFR) + (rx / 0.7));
        backLeft.setPower((powerBL) - (rx / 0.7));
        backRight.setPower((powerBR) + (rx / 0.7));
        
        telemetry.addData("Power_FL", powerFL);
        telemetry.addData("Power_FR", powerFR);
        telemetry.addData("Power_BL", powerBL);
        telemetry.addData("Power_BR", powerBR);
        telemetry.update();
        
        while (opModeIsActive() && (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy())) {
            idle();
        }
        
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    
    public void moveRad (double direction, double revolutions, double rx) {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int tickPerRev = 538;
        
        double powerFL = 0;
        double powerFR = 0;
        double powerBR = 0;
        double powerBL = 0;
        
        double scale = 1;
        
        int targetPos = (int)(revolutions * tickPerRev);
        
        powerFL =  Math.sin(direction + (Math.PI / 4)) * scale;
        powerFR =  Math.cos(direction + (Math.PI / 4)) * scale;
        powerBL =  Math.cos(direction + (Math.PI / 4)) * scale;
        powerBR =  Math.sin(direction + (Math.PI / 4)) * scale;
        
        backLeft.setTargetPosition(targetPos);
        backRight.setTargetPosition(targetPos);
        frontLeft.setTargetPosition(targetPos);
        frontRight.setTargetPosition(targetPos);
        
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeft.setPower((-powerFL ) + (rx / 0.7));
        frontRight.setPower((powerFR) - (rx / 0.7));
        backLeft.setPower((-powerBL) + (rx / 0.7));
        backRight.setPower((powerBR) - (rx / 0.7));
        
        telemetry.addData("Power_FL", powerFL);
        telemetry.addData("Power_FR", powerFR);
        telemetry.addData("Power_BL", powerBL);
        telemetry.addData("Power_BR", powerBR);
        telemetry.update();
        
        while (opModeIsActive() && (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy())) {
            idle();
        }
        
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void move_xy (double x, double y, double rx, double revolutions) {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        final int TICKS_PER_REV = 538;
        
        double r = Math.hypot(x, y);
        double theta = Math.atan2(y, x) - Math.PI / 4;
        
        double powerFL = r * Math.sin(theta);
        double powerFR = r * Math.cos(theta);
        double powerBL = r * Math.cos(theta);
        double powerBR = r * Math.sin(theta);
        
        int targetPos = (int)(revolutions * TICKS_PER_REV);
        
        backLeft.setTargetPosition(targetPos);
        backRight.setTargetPosition(targetPos);
        frontLeft.setTargetPosition(targetPos);
        frontRight.setTargetPosition(targetPos);
        
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        backLeft.setPower(powerBL);
        backRight.setPower(powerBR);
        frontLeft.setPower(powerFL);
        frontRight.setPower(powerFR);
        
        while (opModeIsActive() && (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy())) {
            idle();
        }
        
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
