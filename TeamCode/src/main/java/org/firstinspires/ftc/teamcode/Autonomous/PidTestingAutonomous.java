package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.APIs.ChassisApi;

@Autonomous(name="PID Testing V1.2.7.1")
public class PidTestingAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ChassisApi chassis = new ChassisApi(hardwareMap);

        waitForStart();

        chassis.startDriveForwardPid(21, 0.05, 1, 0.1);

        while(chassis.isActionRunning() && opModeIsActive()) {
            chassis.updatePositionDriveForward();
        }

    }
}
