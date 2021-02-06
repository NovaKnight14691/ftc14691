package org.firstinspires.ftc.teamcode.APIs;

public class FeedForwardLoopApi {

    double p, i, d, motorMaxRpm;

    public FeedForwardLoopApi(double p, double i, double d, double motorMaxRpm) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.motorMaxRpm = motorMaxRpm;
    }

    public double getAdjustedMotorSpeed(double currentMotorSpeed, double currentRpm, double targetRpm) {

        double adjustedCurrentRpm = currentRpm*(1/motorMaxRpm);
        double adjustedTargetRpm = targetRpm*(1/motorMaxRpm);

        PidApi pid = new PidApi(p, i, d);

        return currentMotorSpeed+pid.getOutput(currentRpm, targetRpm);

    }

}
