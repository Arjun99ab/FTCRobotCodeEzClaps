package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="BlueRight", group="Linear Opmode")
@Disabled
public class BlueRight extends AutonomousController {

    double firstDegrees;

    @Override
    public void runOpMode(){

        initRobot();

        while(opModeIsActive()) {
            if(wobblePos == "A") {

                boolean isClamped = false;

                //grip
                powerCRServo(1);
                sleep(1000);

                //lower
                //powerCRServo2(1);
                powerWobbleArm(-0.47);
                sleep(1500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                //powerCRServo2(-0.6);
                //sleep(1000);

                powerWobbleArm(0.45);

                forwardDistance(0.7, 190);

                forward(0);
                sleep(250);

                sidewaysDistanceLeft(0.6, 100);

                powerWobbleArm(-0.47);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                setShooterVelocity();

                sidewaysDistanceRight(0.6, 84); //idk distances

                backwardDistance(0.5, 36); //idk distances
                setShooterVelocity();
                sleep(2000);

                intakeRampShooter();
                sleep(9000);

                forwardDistance(0.4, 27); //idk distance

            } else if(wobblePos == "B") {

                boolean isClamped = false;

                //grip
                powerCRServo(1);
                sleep(1000);

                //lower
                //powerCRServo2(1);
                powerWobbleArm(-0.47);
                sleep(1500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                //powerCRServo2(-0.6);
                //sleep(1000);

                powerWobbleArm(0.45);

                forwardDistance(0.7, 246);

                forward(0);
                sleep(250);

                sidewaysDistanceLeft(0.5, 60);

                powerWobbleArm(-0.47);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                setShooterVelocity();

                sidewaysDistanceRight(0.4, 30); //idk distances

                backwardDistance(0.5, 90.5); //idk distances

                setShooterVelocity();
                sleep(2000);

                intakeRampShooter();
                sleep(9000);

                forwardDistance(0.4, 27); //idk distance

            } else {
                boolean isClamped = false;

                //grip
                powerCRServo(1);
                sleep(1000);

                //lower
                //powerCRServo2(1);
                powerWobbleArm(-0.47);
                sleep(1500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                //powerCRServo2(-0.6);
                //sleep(1000);

                powerWobbleArm(0.45);

                forwardDistance(0.7, 312);

                forward(0);
                sleep(250);

                sidewaysDistanceLeft(0.6, 100);

                powerWobbleArm(-0.47);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                setShooterVelocity();

                backwardDistance(0.5, 157.5); //idk distances

                sidewaysDistanceRight(0.6, 84); //idk distances


                setShooterVelocity();
                sleep(2000);

                intakeRampShooter();
                sleep(9000);

                forwardDistance(0.4, 27); //idk distance

            }

            break;

        }

    }
}
