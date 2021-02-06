package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.PidConstants;

public class ChassisApi {

    // Chassis components
    DcMotor frontLeft, frontRight, rearLeft, rearRight;
    GyroscopeApi gyro;
    StackSlapperApi stackSlapper;
    GrabberApi grabber;

    double wheelSpeeds[] = new double[4];

    double ticksPerWheelRevolution;
    double wheelCircumferenceInInches;

    // Variables used for autonomous actions
    double targetPosition;
    boolean isActionRunning = false;
    double actionP = 0, actionI = 0, actionD = 0;

    /**
     * This creates a new Mecanum API object which can be used to calculate values or drive the robot
     * @param hardwareMap The robot's hardware map
     */
    public ChassisApi(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT_MOTOR_NAME_IN_HARDWARE_MAP);
        frontRight = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT_MOTOR_NAME_IN_HARDWARE_MAP);
        rearLeft = hardwareMap.get(DcMotor.class, Constants.REAR_LEFT_MOTOR_NAME_IN_HARDWARE_MAP);
        rearRight = hardwareMap.get(DcMotor.class, Constants.REAR_RIGHT_MOTOR_NAME_IN_HARDWARE_MAP);
        ticksPerWheelRevolution = Constants.ENCODER_TICKS_PER_DRIVE_WHEEL_REVOLUTION;
        wheelCircumferenceInInches = Math.PI*Constants.DRIVE_WHEEL_DIAMETER_IN_INCHES;

        // Set the motors on the right to run in reverse
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to run with encoders for speed control
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Instantiate our robot's other APIs
        gyro = new GyroscopeApi(hardwareMap);
        stackSlapper = new StackSlapperApi(hardwareMap);
        grabber = new GrabberApi(hardwareMap);
    }

    /**
     * Calculates the speed of wheel given the cartesian inputs. These values can be retrieved by using the get[wheel]Speed() methods.
     * @param x The X value of the left stick (or strafing value)
     * @param y The Y value of the left stick (or forward value)
     * @param rotation The X value of the right stick (or rotation value)
     */
    public void calculateCartesianValues(double x, double y, double rotation) {

        // Reverse the forward and strafe values
        y = -y;
        x = -x;

        wheelSpeeds[0] = y + x + rotation; // Front left
        wheelSpeeds[1] = y - x - rotation; // Front right
        wheelSpeeds[2] = y - x + rotation; // Rear left
        wheelSpeeds[3] = y + x - rotation; // Rear right

        normalizeWheelSpeeds(wheelSpeeds);
    }

    /**
     * Calculates the speed of each wheel given the cartesian inputs, and actually drives the robot based on those values via the HardwareMap fed through the constructor.
     * @param x The X value of the left stick (or strafing value)
     * @param y The Y value of the left stick (or forward value)
     * @param rotation The X value of the right stick (or rotation value)
     */
    public void driveCartesian(double x, double y, double rotation) {

        // Reverse the forward and strafe values
        y = -y;
        x = -x;

        wheelSpeeds[0] = y + x + rotation; // Front left
        wheelSpeeds[1] = y - x - rotation; // Front right
        wheelSpeeds[2] = y - x + rotation; // Rear left
        wheelSpeeds[3] = y + x - rotation; // Rear right

        normalizeWheelSpeeds(wheelSpeeds);

        frontLeft.setPower(wheelSpeeds[0]);
        frontRight.setPower(wheelSpeeds[1]);
        rearLeft.setPower(wheelSpeeds[2]);
        rearRight.setPower(wheelSpeeds[3]);

    }

    /**
     * Drive the robot forward at the speed specified
     * @param power The power to move forward
     */
    public void driveForward(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
    }

    /**
     * Drives forward the specified distance using PID
     * @param distanceToTravelInInches The distance to travel in inches
     * @param p The P value
     * @param i The I value
     * @param d The D value
     * @return Returns true when the target has been reached
     */
    public boolean driveForwardPid(double distanceToTravelInInches, double p, double i, double d) {

        PidApi pid = new PidApi(p, i, d);

        resetEncoders();

        double averageDistanceTraveledInTicks = 0;
        double distanceToTravelInTicks = inchesToTicks(distanceToTravelInInches);

        while(averageDistanceTraveledInTicks != distanceToTravelInInches) {

            averageDistanceTraveledInTicks = (frontLeft.getCurrentPosition()+frontRight.getCurrentPosition()+rearLeft.getCurrentPosition()+rearRight.getCurrentPosition())/4;

            double power = pid.getOutput(averageDistanceTraveledInTicks, distanceToTravelInTicks);

            driveForward(power);

        }

        return true;

    }

    /**
     * Begin driving forward with PID
     * @param distanceToTravelInInches The distance to drive in inches
     * @param p The P value
     * @param i The I value
     * @param d The D value
     */
    public void startDriveForwardPid(double distanceToTravelInInches, double p, double i, double d) {

        isActionRunning = true;

        clearActionPids();

        PidApi pid = new PidApi(p, i, d);

        actionP = p;
        actionI = i;
        actionD = d;

        resetEncoders();

        targetPosition = inchesToTicks(distanceToTravelInInches);

        double power = pid.getOutput(0, targetPosition)* PidConstants.DRIVE_FORWARD_OUTPUT_REDUCTION;

        driveForward(standardizeMotorPower(power));

    }

    /**
     * Start driving forward with PID using the PID values found in PidConstants.java
     * @param distanceToTravelInInches The distance to travel in inches
     */
    public void startDriveForwardPid(double distanceToTravelInInches) {

        isActionRunning = true;

        clearActionPids();

        actionP = PidConstants.DRIVE_FORWARD_P;
        actionI = PidConstants.DRIVE_FORWARD_I;
        actionD = PidConstants.DRIVE_FORWARD_D;

        PidApi pid = new PidApi(actionP, actionI,  actionD);

        resetEncoders();

        targetPosition = inchesToTicks(distanceToTravelInInches);

        double power = pid.getOutput(0, targetPosition)* PidConstants.DRIVE_FORWARD_OUTPUT_REDUCTION;

        driveForward(standardizeMotorPower(power));

    }

    /**
     * Update the current position of the robot for driving forward
     */
    public void updatePositionDriveForward() {

        double currentPosition = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + rearLeft.getCurrentPosition() + rearRight.getCurrentPosition())/4;

        // Stop our action when we arrive at our position
        if(currentPosition <= targetPosition+PidConstants.DRIVE_FORWARD_DEAD_ZONE && currentPosition >= targetPosition-PidConstants.DRIVE_FORWARD_DEAD_ZONE) {
            isActionRunning = false;
            driveForward(0);
            return;
        }

        PidApi pid = new PidApi(actionP, actionI, actionD);

        double power = pid.getOutput(currentPosition, targetPosition)*PidConstants.DRIVE_FORWARD_OUTPUT_REDUCTION;

        driveForward(standardizeMotorPower(power));

    }

    /**
     * Get whether there is an action running
     * @return Whether there is an action running
     */
    public boolean isActionRunning() {
        return isActionRunning;
    }

    /**
     * Clear the PID values
     */
    public void clearActionPids() {
        actionP = 0;
        actionI = 0;
        actionD = 0;
    }

    /**
     * Strafe at the power specified
     * @param power The power to strafe
     */
    public void strafe(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        rearLeft.setPower(-power);
        rearRight.setPower(power);
    }

    /**
     * Strafe using PID
     * @param distanceToTravelInInches The distance to travel in inches
     * @param p The P value
     * @param i The I value
     * @param d The D value
     * @return This method returns true when the robot has reached its target
     */
    public boolean strafePid(double distanceToTravelInInches, double p, double i, double d) {

        boolean areWeTravelingRight;

        if(distanceToTravelInInches > 0) {
            areWeTravelingRight = true;
        } else if(distanceToTravelInInches < 0) {
            areWeTravelingRight = false;
        } else {
            return true;
        }

        PidApi pid = new PidApi(p, i, d);

        resetEncoders();

        double averageDistanceTraveled = 0;

        while(averageDistanceTraveled != distanceToTravelInInches) {

            averageDistanceTraveled = (Math.abs(frontLeft.getCurrentPosition())+Math.abs(frontRight.getCurrentPosition())+Math.abs(rearLeft.getCurrentPosition())+Math.abs(rearRight.getCurrentPosition()))/4;
            double distanceToTravelInTicks = inchesToTicks(distanceToTravelInInches);

            // If we're moving left so the distance traveled needs to be a negative number
            if(!areWeTravelingRight) {
                averageDistanceTraveled = -averageDistanceTraveled;
            }

            double power = pid.getOutput(averageDistanceTraveled, distanceToTravelInTicks);

            strafe(power);

        }

        return true;

    }

    /**
     * Begin the strafe action with PID
     * @param distanceToTravelInInches The distance to strafe in inches. Negative is left, positive is right.
     * @param p The P value to use in the PID loop
     * @param i The I value to use in the PID loop
     * @param d The D value to use in the PID loop
     */
    public void startStrafePid(double distanceToTravelInInches, double p, double i, double d) {
        isActionRunning = true;

        clearActionPids();

        PidApi pid = new PidApi(p, i, d);

        actionP = p;
        actionI = i;
        actionD = d;

        resetEncoders();

        boolean areWeTravelingRight = true;

        if(distanceToTravelInInches > 0) {
            areWeTravelingRight = true;
        } else if(distanceToTravelInInches < 0) {
            areWeTravelingRight = false;
        }

        double averageDistanceTraveled = (Math.abs(frontLeft.getCurrentPosition())+Math.abs(frontRight.getCurrentPosition())+Math.abs(rearLeft.getCurrentPosition())+Math.abs(rearRight.getCurrentPosition()))/4;
        double distanceToTravelInTicks = inchesToTicks(distanceToTravelInInches);

        // If we're moving to the left, the distance traveled needs to be a negative number
        if(!areWeTravelingRight) {
            averageDistanceTraveled = -averageDistanceTraveled;
        }

        double power = pid.getOutput(averageDistanceTraveled, distanceToTravelInTicks);

        strafe(standardizeMotorPower(power));

    }

    /**
     * Update the strafing position for the strafe with PID action
     */
    public void updatePositionStrafe() {

        double currentPosition = (Math.abs(frontLeft.getCurrentPosition())+Math.abs(frontRight.getCurrentPosition())+Math.abs(rearLeft.getCurrentPosition())+Math.abs(rearRight.getCurrentPosition()))/4;

        // Stop our action when we arrive at our position
        if(currentPosition == targetPosition) {
            isActionRunning = false;
            driveForward(0);
            return;
        }

        PidApi pid = new PidApi(actionP, actionI, actionD);

        double power = pid.getOutput(currentPosition, targetPosition);

        strafe(standardizeMotorPower(power));

    }

    /**
     * Reset all of the drive motor encoders
     */
    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Changes inches to ticks
     * @param inches The amount of inches
     * @return That amount of inches in ticks
     */
    private double inchesToTicks(double inches) {
        return (inches/wheelCircumferenceInInches)*ticksPerWheelRevolution;
    }

    /**
     * Change ticks to inches
     * @param ticks The amount of ticks
     * @return That amount of ticks as inches
     */
    private double ticksToInches(double ticks) {
        return (ticks/ticksPerWheelRevolution)*wheelCircumferenceInInches;
    }

    /**
     * Normalize the wheel speeds by making the minimum and maximum 0
     * @param wheelSpeeds The normalized wheel speeds
     */
    private void normalizeWheelSpeeds(double[] wheelSpeeds) {

        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++) {

            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }

        }

        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }

    }

    /**
     * Get the calculated speed of the front left wheel
     * @return The speed of the front left wheel
     */
    public double getFrontLeftSpeed() {
        return wheelSpeeds[0];
    }

    /**
     * Get the calculated speed of the front right wheel
     * @return The speed of the front right wheel
     */
    public double getFrontRightSpeed() {
        return wheelSpeeds[1];
    }

    /**
     * Get the calculated speed of the rear left wheel
     * @return The speed of the rear left wheel
     */
    public double getRearLeftSpeed() {
        return wheelSpeeds[2];
    }

    /**
     * Get the calculated speed of the rear right wheel
     * @return The speed of the rear right wheel
     */
    public double getRearRightSpeed() {
        return wheelSpeeds[3];
    }

    /**
     * Standardizes the motor's power
     * @param power The power to standardize
     * @return
     */
    public double standardizeMotorPower(double power) {
        if (power > 1) {
            return 1;
        } else if(power < -1) {
            return -1;
        } else {
            return power;
        }
    }

    /**
     * Gets the heading of the robot and normalizes it (E.G. 720 will be changed to 0)
     * @return The robots current heading
     */
    public double getHeading() {
        //TODO change this to the proper gyro axis
        return gyro.getStandardizedX();
    }

    /**
     * Gets the heading of the robot without normalizing it (E.G. 720 won't be changed to 0)
     * @return The robots current heading
     */
    public double getRawHeading() {
        //TODO change this to the proper gyro axis
        return gyro.getRawX();
    }

    /**
     * Close the grabber's claw
     */
    public void closeClaw() {
        grabber.setClawPosition(Constants.CLAW_CLOSED_POSITION);
    }

    /**
     * Open the grabber's claw
     */
    public void openClaw() {
        grabber.setClawPosition(Constants.CLAW_OPEN_POSITION);
    }

    /**
     * Set the claw to a certain position
     * @param position The new position as a value between 0 and 1
     */
    public void setClawPosition(double position) {
        grabber.setClawPosition(position);
    }

    /**
     * Gets the grabber claw's current position
     * @return A value between 0 and 1 that represents the claw's position
     */
    public double getClawPosition() {
        return grabber.getClawPosition();
    }

    /**
     * Sets the power of the grabber motor to the input times the constant speed reduction
     * @param power The power to set the motor to
     */
    public void setGrabberPower(double power) {
        grabber.setGrabberPower(Constants.GRABBER_MOTOR_SPEED_REDUCTION*power);
    }

    /**
     * Sets the power of the stack slapper motor
     * @param power The power to set the motor to
     */
    public void setStackSlapperPower(double power) {
        stackSlapper.setPower(power);
    }

    /**
     * Gets the current speed of the stack slapper motor
     * @return The stack slapper's current speed
     */
    public double getStackSlapperPower() {
        return stackSlapper.getPower();
    }

    /**
     * Checks if a ring is currently detected by  the stack slapper
     * @return Whether a ring is detected or not
     */
    public boolean isRingDetected() {
        return stackSlapper.isRingDetected();
    }

    /**
     * Get the front left motor
     * @return The front left motor object
     */
    public DcMotor getFrontLeftMotor() {
        return frontLeft;
    }

    /**
     * Get the front right motor
     * @return The rear right motor object
     */
    public DcMotor getFrontRightMotor() {
        return frontRight;
    }

    /**
     * Get the rear left motor
     * @return The rear left motor object
     */
    public DcMotor getRearLeftMotor() {
        return rearLeft;
    }

    /**
     * Get the rear right motor
     * @return The rear right motor object
     */
    public DcMotor getRearRightMotor() {
        return rearRight;
    }

    /**
     * Get the stack slapper's current position
     * @return The stack slapper's position in ticks
     */
    public double getStackSlapperPosition() {
        return stackSlapper.getCurrentPosition();
    }

    /**
     * Gets the grabber's current position
     * @return The grabber's position in ticks
     */
    public double getGrabberPosition() {
        return grabber.getGrabberPosition();
    }

}
