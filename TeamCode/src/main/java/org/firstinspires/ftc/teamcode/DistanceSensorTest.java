package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="DistanceSensorTest", group="Linear Opmode")
//@Disabled
public class DistanceSensorTest extends AutonomousController {

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        while (opModeIsActive()) {
            //telemetry.addData("deviceName", distanceSensorLeft.getDeviceName());
            //telemetry.addData("Left Range", String.format("%.01f cm", distanceSensorLeft.getDistance(DistanceUnit.CM)));
            //telemetry.addData("deviceName", distanceSensorRight.getDeviceName());
            //telemetry.addData("Right Range", String.format("%.01f cm", distanceSensorRight.getDistance(DistanceUnit.CM)));
            telemetry.addData("deviceName", distanceSensorFront.getDeviceName());
            telemetry.addData("Front Range", String.format("%.01f cm", distanceSensorFront.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}
