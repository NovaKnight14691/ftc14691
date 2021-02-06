package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.APIs.ChassisApi;
import org.firstinspires.ftc.teamcode.Constants.PidConstants;

@Autonomous(name="Deliver to C and Navigate V1.2")
public class DeliverAndNavigatePID extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ChassisApi chassis = new ChassisApi(hardwareMap);

        waitForStart();

        // Drive to target zone C
        telemetry.addLine("Status: Reversing to target C");
        telemetry.addLine("PID is ENABLED");
        telemetry.update();
        chassis.startDriveForwardPid(-110);
        while(chassis.isActionRunning() && opModeIsActive()) {
            chassis.updatePositionDriveForward();
        }

        // Wait 2 seconds
        long waitStartTimeInMillis = System.currentTimeMillis();
        while(System.currentTimeMillis() < waitStartTimeInMillis+2000 && opModeIsActive()) {
            telemetry.addLine("Status: Waiting");
            long timeRemainingInMillis = (waitStartTimeInMillis+2000)-System.currentTimeMillis();
            telemetry.addLine("Time remaining: " + timeRemainingInMillis + " millis");
            telemetry.update();
        }

        // Reverse to launch line
        telemetry.addLine("Status: Driving to launch line");
        telemetry.addLine("PID is ENABLED");
        telemetry.update();
        chassis.startDriveForwardPid(39);
        while(chassis.isActionRunning() && opModeIsActive()) {
            chassis.updatePositionDriveForward();
        }

        telemetry.addLine("Status: Program finished");
        telemetry.update();

    }
}
