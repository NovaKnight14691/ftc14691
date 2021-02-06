package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroscopeApi {
    private BNO055IMU imu;
    private float xAngle, yAngle, zAngle;


    public GyroscopeApi(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode    = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);

        this.xAngle = this.yAngle = this.zAngle = 0;
    }

    public void update() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        float startX = this.xAngle, startY = this.yAngle, startZ = this.zAngle;

        this.xAngle = getRelativeClosestAngle(this.xAngle, orientation.firstAngle);
        this.yAngle = getRelativeClosestAngle(this.yAngle, orientation.secondAngle);
        this.zAngle = getRelativeClosestAngle(this.zAngle, orientation.thirdAngle);
    }

    private float getRelativeClosestAngle(float currentAngle, float targetAngle) {
        while(targetAngle - currentAngle >= 360) {
            targetAngle -= 360;
        }

        while(targetAngle - currentAngle <= -360) {
            targetAngle += 360;
        }

        if(targetAngle != currentAngle) {
            float negativeDirectionTargetAngle = 0;
            float positiveDirectionTargetAngle = 0;

            if(targetAngle < currentAngle) {
                negativeDirectionTargetAngle = targetAngle;
                positiveDirectionTargetAngle = targetAngle + 360;
            } else {
                negativeDirectionTargetAngle = targetAngle - 360;
                positiveDirectionTargetAngle = targetAngle;
            }
            float negativeDirectionDistance = negativeDirectionTargetAngle - currentAngle;
            float positiveDirectionDistance = positiveDirectionTargetAngle - currentAngle;

            if(Math.abs(positiveDirectionDistance) < Math.abs(negativeDirectionDistance)) return positiveDirectionTargetAngle;
            else return negativeDirectionTargetAngle;
        }

        return targetAngle;
    }

    public float getRawX() {
        return xAngle;
    }

    public float getRawY() {
        return yAngle;
    }

    public float getRawZ() {
        return zAngle;
    }

    public float getStandardizedX() {
        return xAngle%360;
    }

    public float getStandardizedY() {
        return  yAngle%360;
    }

    public float getStandardizedZ() {
        return xAngle%360;
    }
}