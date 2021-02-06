package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDeviceCloseOnTearDown;

import org.firstinspires.ftc.teamcode.APIs.ChassisApi;
import org.firstinspires.ftc.teamcode.APIs.GyroscopeApi;
import org.firstinspires.ftc.teamcode.APIs.StackSlapperApi;

@TeleOp(name="Telemetry and Manual Control")
/**
 * This opmode allows you to see the output of every sensor on the robot and to control the motors individually
 */
public class TelemetryAndManualControl extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        String buttonCombination = "Left Bumper + Right Bumper + A";

        telemetry.addLine("Status: Initializing robot...");
        telemetry.update();

        ChassisApi chassis = new ChassisApi(hardwareMap);
        GyroscopeApi gyro = new GyroscopeApi(hardwareMap);
        StackSlapperApi stackSlapper = new StackSlapperApi(hardwareMap);

        telemetry.addLine("Status: Robot Ready");
        telemetry.addLine("Note: Program will only output data unless you do the following button combination:");
        telemetry.addLine(buttonCombination);
        telemetry.update();

        waitForStart();

        boolean outputOnlyMode = true;
        boolean comboPreviouslyPressed = false;

        while(opModeIsActive()) {

            // Toggle output mode
            if(gamepad1.a && gamepad1.left_bumper && gamepad1.right_bumper && !comboPreviouslyPressed) {
                outputOnlyMode = !outputOnlyMode;
                comboPreviouslyPressed = true;
            } else if(!(gamepad1.a && gamepad1.left_bumper && gamepad1.right_bumper)) {
                comboPreviouslyPressed = false;
            }

            if(outputOnlyMode) {

                telemetry.addLine("===CHASSIS===");
                telemetry.addLine("Front Left Encoder: " + chassis.getFrontLeftMotor().getCurrentPosition());
                telemetry.addLine("Front Right Encoder: " + chassis.getFrontRightMotor().getCurrentPosition());
                telemetry.addLine("Rear Left Encoder: " + chassis.getRearLeftMotor().getCurrentPosition());
                telemetry.addLine("Rear Right Encoder: " + chassis.getRearRightMotor().getCurrentPosition());
                telemetry.addLine("Gyro: X: " + gyro.getRawX() + " Y (heading): " + gyro.getRawY() + " Z: " + gyro.getRawZ());

                telemetry.addLine("===STACK SLAPPER===");
                telemetry.addLine("Stack Slapper Encoder: " + chassis.getStackSlapperPosition());
                telemetry.addLine("Touch 1: " + stackSlapper.isSensor1Pressed() + " Touch 2: " + stackSlapper.isSensor2Pressed() + " Touch 3: " + stackSlapper.isSensor3Pressed() + " Touch 4: " + stackSlapper.isSensor4Pressed());

                telemetry.addLine("===GRABBER===");
                telemetry.addLine("Grabber Encoder: " + chassis.getGrabberPosition());
                telemetry.update();


            } else {

                telemetry.addLine("Isaac ran out of time while making this mode so this isn't finished.");
                telemetry.addLine("Switch back to output only mode by doing");
                telemetry.addLine(buttonCombination);
                telemetry.update();

            }

        }


    }
}
