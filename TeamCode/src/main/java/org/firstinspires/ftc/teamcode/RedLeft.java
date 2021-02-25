package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="RedLeft", group="Linear Opmode")
//@Disabled
public class RedLeft extends AutonomousController {

    @Override
    public void runOpMode(){

        initRobot();

        while(opModeIsActive()) {


            if(wobblePos == "A") {

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

                forwardDistance(0.8, 190);

                forward(0);
                sleep(100);

                //go to A
                sidewaysDistanceRight(0.8, 70);

                //lower
                //powerCRServo2(0.8);

                powerWobbleArm(-0.47);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                setShooterVelocity();

                //navigate to shooter position
                backwardDistance(0.5, 35.25);
                sidewaysDistanceLeft(0.5, 36);

                setShooterVelocity();
                sleep(2000);

                intakeRampShooter();
                sleep(10000);

                //park at white line
                forwardDistance(0.8, 27);

            } else if(wobblePos == "B") {

                boolean isClamped = false;

                //grip
                powerCRServo(1);
                sleep(1000);

                //lower
                //powerCRServo2(0.6);
                sleep(500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                //powerCRServo2(-0.6);
                sleep(1000);

                forwardDistance(0.6, 246);

                forward(0);
                sleep(100);

                setShooterVelocity();

                //lower
                //powerCRServo2(0.8);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                //navigate to shooter position
                backwardDistance(0.5, 90.5); //idk distances
                sidewaysDistanceRight(0.3, 37); //idk distances

                intakeRampShooter();
                sleep(10000);

                //park at white line
                forwardDistance(0.4, 27);

            } else {
                boolean isClamped = false;

                //grip
                powerCRServo(1);
                sleep(1000);

                //lower
                powerCRServo2(0.6);
                sleep(500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                powerCRServo2(-0.6);
                sleep(1000);

                forwardDistance(0.6, 312);

                //move in position to place goal
                sidewaysDistanceRight(0.7, 60.96);

                forward(0);
                sleep(100);

                setShooterVelocity();

                //lower
                powerCRServo2(1);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                //navigate to shooter position
                backwardDistance(0.6, 157.5);
                sidewaysDistanceLeft(0.4, 20);

                intakeRampShooter();
                sleep(8000);

                //park at white line
                forwardDistance(0.4, 27);

            }

            break;

        }

    }
}
