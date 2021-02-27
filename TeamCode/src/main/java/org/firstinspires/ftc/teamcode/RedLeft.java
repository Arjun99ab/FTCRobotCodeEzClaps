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
                powerCRServo2(-0.8);
                sleep(1500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                //powerCRServo2(-0.6);
                //sleep(1000);

                //powerWobbleArm(0.45);

                forwardDistance(0.8, 190);

                forward(0);
                sleep(100);

                //go to A
                sidewaysDistanceRight(0.8, 70);

                //lower
                //powerCRServo2(0.8);

                //powerWobbleArm(-0.47);
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

                sidewaysDistanceLeft(0.5, 50);

            } else if(wobblePos == "B") {

                boolean isClamped = false;

                //grip
                powerCRServo(1);
                sleep(1000);

                //lower
                powerCRServo2(-0.8);
                sleep(500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                powerCRServo2(0.8);
                sleep(1000);

                setShooterVelocity();

                forwardDistance(0.6, 246);

                forward(0);
                sleep(100);

                //lower
                powerCRServo2(-0.8);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                //navigate to shooter position
                backwardDistance(0.5, 90.5); //idk distances
                sidewaysDistanceRight(0.45, 37.5); //idk distances

                intakeRampShooter();
                sleep(5000);

                //park at white line
                forwardDistance(0.4, 27);

                sidewaysDistanceRight(0.5, 50);

            } else {
                boolean isClamped = false;

                //grip
                powerCRServo(1);
                sleep(1000);

                //lower
                powerCRServo2(-0.8);
                sleep(500);

                //grip
                powerCRServo(1);
                sleep(500);

                //raise
                powerCRServo2(0.8);
                sleep(1000);

                setShooterVelocity();

                forwardDistance(0.6, 312);

                forward(0);
                sleep(100);

                sidewaysDistanceRight(0.5, 60.96);

                //lower
                powerCRServo2(-0.8);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                //navigate to shooter position
                backwardDistance(0.5, 155.5); //idk distances
                sidewaysDistanceRight(0.45, 40.5); //idk distances

                intakeRampShooter();
                sleep(5000);

                //park at white line
                forwardDistance(0.4, 27);

                sidewaysDistanceRight(0.5, 50);

            }

            break;

        }

    }
}
