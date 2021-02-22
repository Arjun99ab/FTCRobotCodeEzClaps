package org.firstinspires.ftc.teamcode;

/**
 @author Shriyana Grande,
 @author Arjoon
 @version 1.0
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="TeleOp Controller", group="Linear Opmode")
@Disabled
public abstract class TeleOpController extends BaseController {

    public void initRobot(){
        super.initRobot();

        waitForStart();
    }
    public void setShooterVelocityInput(double shooterVelocity){
        shooter.setVelocity(shooterVelocity);
    }
    public void moveDistanceSensorPowershotSideways(double power, double distance) {
        if(currentDistance > (distance + 3)) {
            //driveUntilDistanceCMLEFT(power, distance);
        } else if(currentDistance < (distance - 3)){
            //driveUntilDistanceCMRIGHT(power, distance);
        } else {
            applyBrake();
            forward(0);
        }
    }

    public void moveDistanceSensorHighGoalSideways(double power, double distance) {
        if(currentDistance > (distance + 3)) {
            //driveUntilDistanceCMRIGHT2(power, distance);
        } else if(currentDistance < (distance - 3)){
            //driveUntilDistanceCMLEFT2(power, distance);
        } else {
            applyBrake();
            forward(0);
        }
    }
    public void moveDistanceSensorForward(double power, double distance) {
        if(currentDistance2 > (distance + 1)) {
            driveUntilDistanceCMFORWARD(power, distance);
        } else if(currentDistance2 < (distance - 1)) {
            driveUntilDistanceCMBACKWARD(power, distance);
        } else {
            //applyBrake();
            forward(0);
        }
    }


}
