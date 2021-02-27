package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="RedRightHighGoal", group="Linear Opmode")
//@Disabled
public class RedRightHighGoal extends AutonomousController {

    double firstDegrees;

    @Override
    public void runOpMode(){

        initRobotWithoutVision();


        while(opModeIsActive()) {

            setShooterVelocity();

            applyBrake();

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

            sidewaysDistanceRight(0.3, 32);

            forwardDistance(0.9, 159);
            //forwardDistance(0.3, 20);

            sidewaysDistanceLeft(0.5, 70);

            intakeRampShooter();
            sleep(5000);

            //park at white line
            forwardDistance(0.4, 128);

            //change to move out of the way
            sidewaysDistanceRight(0.5, 35);

            //lower
            powerCRServo2(-0.8);
            sleep(1000);

            //ungrip
            powerCRServo(-0.6);
            sleep(1000);

            backwardDistance(0.5, 145);


        break;

        }

    }
}
