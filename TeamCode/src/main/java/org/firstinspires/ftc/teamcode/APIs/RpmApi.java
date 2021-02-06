package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RpmApi {

    boolean hasRpmBeenInitialized = false;

    PidApi pid;
    double encoderTicksPerMotorRevolution;
    DcMotor motor;

    // These variables are for the RPM calculating method
    double rpmLoopPreviousTimeInMillis;
    double rpmLoopPreviousTickCount;
    double previousRpm;
    double rpmLoopMillisToWait = 500;

    /**
     * Instantiates a new RpmApi object
     * @param motor The motor which we are getting the RPM of
     * @param encoderTicksPerMotorRevolution The amount of ticks per output rotation
     */
    public RpmApi(DcMotor motor, double encoderTicksPerMotorRevolution) {
        this.motor = motor;
        this.encoderTicksPerMotorRevolution = encoderTicksPerMotorRevolution;
    }

    /**
     * Instantiates a new RpmApi object
     * @param motor The motor which we are getting the RPM of
     * @param encoderTicksPerMotorRevolution The amount of ticks per output rotation
     * @param timeToWaitBetweenMeasurementsInMillis The amount of time to wait between each RPM measurement, in millis. Defeault is 500
     */
    public RpmApi(DcMotor motor, double encoderTicksPerMotorRevolution, double timeToWaitBetweenMeasurementsInMillis) {
        this.motor = motor;
        this.encoderTicksPerMotorRevolution = encoderTicksPerMotorRevolution;
        rpmLoopMillisToWait = timeToWaitBetweenMeasurementsInMillis;
    }

    /**
     * Initializes the RPM counter. Note: This must be run before running the getRpm method
     * @param currentTickCount
     */
    public void initializeRpmCounter(double currentTickCount) {
        previousRpm = 0;
        rpmLoopPreviousTimeInMillis = System.currentTimeMillis();
        rpmLoopPreviousTickCount = currentTickCount;
        hasRpmBeenInitialized = true;
    }

    /**
     * Gets the current RPM of the motor
     * @param currentTickCount The current position of the motor
     * @return
     */
    public double getRpm(double currentTickCount) {

        if(!hasRpmBeenInitialized) {
            return 0;
        }

        double currentTimeInMillis = System.currentTimeMillis();

        if(currentTimeInMillis >= rpmLoopPreviousTimeInMillis + rpmLoopMillisToWait) {

            double previousRotation = rpmLoopPreviousTickCount /encoderTicksPerMotorRevolution;
            double currentRotation = currentTickCount/encoderTicksPerMotorRevolution;
            double changeInRotation = 0;

            if(previousRotation > currentRotation) {
                changeInRotation = previousRotation-currentRotation;
            } else if(currentRotation > previousRotation) {
                changeInRotation = currentRotation-previousRotation;
            }

            double changeInTimeInSeconds = (currentTimeInMillis- rpmLoopPreviousTimeInMillis)/1000;

            double rotationsPerMinute = (changeInRotation*60)/changeInTimeInSeconds;

            previousRpm = rotationsPerMinute;

            rpmLoopPreviousTimeInMillis = currentTimeInMillis;

            rpmLoopPreviousTickCount = currentTickCount;

            return rotationsPerMinute;

        } else {
            return previousRpm;
        }

    }

}
