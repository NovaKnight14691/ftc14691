package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.APIs.GrabberApi;
import org.firstinspires.ftc.teamcode.APIs.ChassisApi;
import org.firstinspires.ftc.teamcode.Constants.Constants;

@TeleOp(name="Main Teleop V2.6.3")
public class MainTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {

        ChassisApi chassis = new ChassisApi(hardwareMap);

        boolean debugMode = true;

        // Booleans for button pressed
        boolean previousA = false;

        waitForStart();

        chassis.closeClaw();

        while(opModeIsActive()) {

            // Drive based on joystick inputs
            chassis.driveCartesian(0.5*gamepad1.left_stick_x, 0.5*gamepad1.left_stick_y, 0.5*gamepad1.right_stick_x);

            // Set the power of the grabber
            chassis.setGrabberPower(gamepad1.right_trigger-gamepad1.left_trigger);

            // Set the grabber power
            if(gamepad1.dpad_up && !gamepad1.dpad_down) {
                chassis.setStackSlapperPower(0.2);
            } else if(gamepad1.dpad_down && !gamepad1.dpad_up) {
                chassis.setStackSlapperPower(-0.2);
            } else {
                chassis.setStackSlapperPower(0);
            }

            // Do some logic for A being pressed and toggling the position of the claw
            if(gamepad1.a && previousA == false) {
                if(chassis.getClawPosition() < Constants.CLAW_OPEN_POSITION + Constants.CLAW_DEAD_ZONE && chassis.getClawPosition() > Constants.CLAW_OPEN_POSITION-Constants.CLAW_DEAD_ZONE) {
                    chassis.closeClaw();
                    previousA = true;
                } else {
                    chassis.openClaw();
                    previousA = true;
                }
            } else if(!gamepad1.a) {
                previousA = false;
            }

            if(debugMode) {
                telemetry.addLine(chassis.getFrontLeftSpeed() + "(-----)" + chassis.getFrontRightSpeed());
                telemetry.addLine("|       |");
                telemetry.addLine("|       |");
                telemetry.addLine("|       |");
                telemetry.addLine(chassis.getRearLeftSpeed() + "(-----)" + chassis.getRearRightSpeed());
                telemetry.addLine("X: " + gamepad1.left_stick_x + " Y: " + gamepad1.left_stick_y + " ROT: " + gamepad1.right_stick_x);
            }
            telemetry.update();
        }

    }
}
