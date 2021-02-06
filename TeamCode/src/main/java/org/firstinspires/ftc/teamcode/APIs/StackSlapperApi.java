package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Constants.Constants;

public class StackSlapperApi {

    DcMotor stackSlapperMotor;
    TouchSensor sensor1, sensor2, sensor3, sensor4;

    String stackSlapperMotorNameInHardwareMap = "stackSlapper";

    double encoderTicksPerRotation;

    /**
     * Instantiate a new stack slapper API object
     * @param hardwareMap The robot's hardware map
     */
    public StackSlapperApi(HardwareMap hardwareMap) {
        stackSlapperMotor = hardwareMap.get(DcMotor.class, stackSlapperMotorNameInHardwareMap);
        stackSlapperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderTicksPerRotation = Constants.STACK_SLAPPER_MOTOR_ENCODER_TICKS;
    }

    /**
     * Reset the stack slapper motor's encoders
     */
    public void resetEncoder() {
        stackSlapperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stackSlapperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set the power of the stack slapper motor
     * @param power The power to set the stack slapper motor to
     */
    public void setPower(double power) {
        stackSlapperMotor.setPower(power);
    }

    /**
     * Gets the power of the stack slapper motor
     * @return The current power of the stack slapper motor
     */
    public double getPower() {
        return stackSlapperMotor.getPower();
    }

    /**
     * Determines if a ring is detected
     * @return Whether a ring is detected or not
     */
    public boolean isRingDetected() {
        return sensor1.isPressed() || sensor2.isPressed() || sensor3.isPressed() || sensor4.isPressed();
    }

    /**
     * Gets the current position of the stack slapper motor
     * @return The current position in rotations
     */
    public double getCurrentPosition() {
        return stackSlapperMotor.getCurrentPosition()/encoderTicksPerRotation;
    }

    public boolean isSensor1Pressed() {
        return sensor1.isPressed();
    }

    public boolean isSensor2Pressed() {
        return sensor2.isPressed();
    }

    public boolean isSensor3Pressed() {
        return sensor3.isPressed();
    }

    public boolean isSensor4Pressed() {
        return sensor4.isPressed();
    }

}
