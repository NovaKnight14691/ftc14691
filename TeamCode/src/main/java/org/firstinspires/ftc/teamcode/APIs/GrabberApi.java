package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GrabberApi {

    DcMotor grabberMotor;
    Servo clawServo;

    String grabberMotorNameInHardwareMap = "grabber";
    String clawServoNameInHardwareMap = "claw";

    boolean shouldCurrentProcessBeCancelled = false;

    /**
     * Instantiates a new GrabberApi object
     * @param hardwareMap The robot's hardware map
     */
    public GrabberApi(HardwareMap hardwareMap) {
        grabberMotor = hardwareMap.get(DcMotor.class, grabberMotorNameInHardwareMap);
        clawServo = hardwareMap.get(Servo.class, clawServoNameInHardwareMap);
        grabberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the grabber motor's power
     * @param power The power to set the grabber motor to
     */
    public void setGrabberPower(double power) {
        grabberMotor.setPower(power);
    }

    /**
     * Gets the current power of the grabber motor
     * @return The current power of the grabber motor
     */
    public double getGrabberPower() {
        return grabberMotor.getPower();
    }

    /**
     * Resets the grabber encoder
     */
    public void resetGrabberEncoder() {
        grabberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //TODO fix this grabber code because we know this won't work, since it didn't with the drive train. We'll get stuck in a while loop.
    /**
     * Runs the grabber motor to the specified position using PID
     * @param positionInTicks The position to move to in ticks
     * @param p The P value
     * @param i The I value
     * @param d The D value
     * @return
     */
    public boolean runGrabberToPosition(double positionInTicks, double p, double i, double d) {
        PidApi pid = new PidApi(p, i, d);
        while(grabberMotor.getCurrentPosition() != positionInTicks && !shouldCurrentProcessBeCancelled) {
            grabberMotor.setPower(pid.getOutput(grabberMotor.getCurrentPosition(), positionInTicks));
        }
        if(shouldCurrentProcessBeCancelled) {
            shouldCurrentProcessBeCancelled = false;
        }
        return true;
    }

    /**
     * Sets the position of the claw
     * @param positionInDegrees The position to set the claw to
     */
    public void setClawPosition(double positionInDegrees) {
        clawServo.setPosition(positionInDegrees);
    }

    /**
     * Gets the position of the claw
     * @return The position of the claw
     */
    public double getClawPosition() {
        return clawServo.getPosition();
        //TODO add some logic to this API that allows for entering encoder ticks per rotation and returning rotations rather than total encoder ticks
    }

    /**
     * Gets the position of the grabber
     * @return The position of the grabber in ticks
     */
    public double getGrabberPosition() {
        return grabberMotor.getCurrentPosition();
    }

    /**
     * Cancels the process that's currently running on the grabber motor
     */
    public void cancelGrabberMotorCurrentProcess() {
        shouldCurrentProcessBeCancelled = true;
    }

}
