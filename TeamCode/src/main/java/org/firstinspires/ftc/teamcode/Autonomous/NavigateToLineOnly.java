package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.APIs.TankDriveApi;

@Autonomous(name = "Only drive to line | V1.2")
public class NavigateToLineOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        TankDriveApi drive = new TankDriveApi(hardwareMap, 0, 0);

        waitForStart();

        // Drive forward
        double startTimeOne = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTimeOne+15500 && opModeIsActive()) {
            drive.driveTank(-0.5, -0.5);
        }
        drive.stopMotors();

        // Wait
        double startTimeTwo = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTimeTwo + 2000 && opModeIsActive()) {

        }

        // Drive backwards
        double startTimeThree = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTimeThree+5000 && opModeIsActive()) {
            drive.driveTank(0.5, 0.5);
        }

        drive.stopMotors();

    }

}
