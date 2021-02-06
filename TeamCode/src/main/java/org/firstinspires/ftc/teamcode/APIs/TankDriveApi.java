package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TankDriveApi {

    DcMotor frontLeft, frontRight, rearLeft, rearRight;

    // The names of the motors in the hardware map
    String frontLeftMotorNameInHardwareMap = "frontLeft";
    String frontRightMotorNameInHardwareMap = "frontRight";
    String rearLeftMotorNameInHardwareMap = "rearLeft";
    String rearRightMotorNameInHardwareMap = "rearRight";

    double ticksPerWheelRevolution;
    double wheelCircumferenceInInches;

    public TankDriveApi(HardwareMap hardwareMap, double ticksPerWheelRevolution, double wheelDiameterInInches) {
        frontLeft = hardwareMap.get(DcMotor.class, frontLeftMotorNameInHardwareMap);
        frontRight = hardwareMap.get(DcMotor.class, frontRightMotorNameInHardwareMap);
        rearLeft = hardwareMap.get(DcMotor.class, rearLeftMotorNameInHardwareMap);
        rearRight = hardwareMap.get(DcMotor.class, rearRightMotorNameInHardwareMap);
        this.ticksPerWheelRevolution = ticksPerWheelRevolution;
        wheelCircumferenceInInches = Math.PI*wheelDiameterInInches;
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public TankDriveApi(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight, double ticksPerWheelRevolution, double wheelDiameterInInches) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.ticksPerWheelRevolution = ticksPerWheelRevolution;
        wheelCircumferenceInInches = Math.PI*wheelDiameterInInches;
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveForward(double power) {
        frontLeft.setPower(0.5*power);
        frontRight.setPower(0.5*power);
        rearLeft.setPower(0.5*power);
        rearRight.setPower(0.5*power);
    }

    public void driveTank(double leftPower, double rightPower) {
        frontLeft.setPower(0.5*leftPower);
        rearLeft.setPower(0.5*leftPower);
        frontRight.setPower(0.5*rightPower);
        rearRight.setPower(0.5*rightPower);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

}
