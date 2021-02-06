package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.APIs.ChassisApi;

public class BasicAutonomous extends LinearOpMode {

    double mecanumWheelDiameterInInches =2.952756;

    double strafeP = 0;
    double strafeI = 0;
    double strafeD = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        ChassisApi drive = new ChassisApi(hardwareMap);

        waitForStart();

        // magically shoot

//        while(!drive.strafePid(50,0, 0, 0)) {
//            telemetry.addLine("Strafing 50 inches with PID enabled");
//            telemetry.update();
//        }
//
//        wait(500);
//
//        while(!drive.driveForwardPid(36, 0, 0, 0)) {
//            telemetry.addLine("Driving forward 36 inches with PID enabled");
//            telemetry.update();
//        }
//
//        int ringsPresent = -1;
//
//        // Should be slapping the stack instead of waiting here
//        wait(5000);
//
//        if(ringsPresent == 1) {
//
//            // Drive until we hit the line, then follow the line and strafe left
//
//            while(!drive.driveForwardPid(20, 0, 0, 0)) {
//                telemetry.addLine("Driving towards target zone B with PID enabled");
//                telemetry.update();
//            }
//
//            // Release wobbler
//
//            while(!drive.strafePid(-24, 0, 0, 9)) {
//                telemetry.addLine("Strafing left 24 inches with PID enabled");
//                telemetry.update();
//            }
//
//            // Drive back to line
//
//        } else if(ringsPresent == 0 || ringsPresent == 4) {
//
//            // Drive towards the proper target zone
//            if(ringsPresent == 0) {
//                while(!drive.driveForwardPid(20, 0, 0, 0)) {
//                    telemetry.addLine("Driving towards target zone A with PID enabled");
//                    telemetry.update();
//                }
//            } else {
//                while(!drive.driveForwardPid(72, 0, 0, 0)) {
//                    telemetry.addLine("Driving towards target zone C with PID enabled");
//                    telemetry.update();
//                }
//            }
//
//            // Release wobbler here
//
//            while(!drive.strafePid(-60, 0, 0, 0)) {
//                telemetry.addLine("Strafing left 60 inches with PID enabled");
//                telemetry.update();
//            }
//
//            // Reverse until we hit the line, then reverse the set amount until we hit the wall (but park for now because I haven't written the rest yet)
//
//        } else {
//            telemetry.addLine("ERROR: Failed to find stack");
//            telemetry.addLine("Program halting");
//            telemetry.update();
//        }

    }

    public void wait(double millisToWait) {

        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime+millisToWait) {
            double timeRemaining = (startTime+millisToWait)-System.currentTimeMillis();
            telemetry.addLine("Waiting for " + millisToWait + " millis");
            telemetry.addLine("Millis remaining: " + timeRemaining);
            telemetry.update();
        }

    }

}
