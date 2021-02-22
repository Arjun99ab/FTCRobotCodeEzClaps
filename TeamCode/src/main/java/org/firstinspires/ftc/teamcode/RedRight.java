package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="RedRight", group="Linear Opmode")
//@Disabled
public class RedRight extends AutonomousController {

    double firstDegrees;

    @Override
    public void runOpMode()
    {
        initRobot();


            while(opModeIsActive()) {
                if(wobblePos == "B") {

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

                    sidewaysDistanceLeft(0.5, 80);

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

                } else if(wobblePos == "C") {
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

                    //continue gripping
                    powerCRServo(1);
                    sleep(500);

                    //lower
                    powerCRServo2(-0.8);
                    sleep(1000);

                    //ungrip
                    powerCRServo(-0.6);
                    sleep(1000);

                    //navigate to shooter position
                    backwardDistance(0.6, 155.5);
                    sidewaysDistanceLeft(0.3, 40.5);

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
                    sleep(700);

                    //grip
                    powerCRServo(1);
                    sleep(500);

                    //raise
                    powerCRServo2(0.8);
                    sleep(1000);

                    setShooterVelocity();
                    forwardDistance(0.6, 190);
                    //forwardDistance(0.3, 20);

                    forward(0);
                    sleep(100);

                    //lower
                    powerCRServo2(-0.8);
                    sleep(1000);

                    //ungrip
                    powerCRServo(-0.6);
                    sleep(1000);


                    //navigate to shooter position
                    backwardDistance(0.4, 36);
                    sidewaysDistanceLeft(0.3, 29);

                    //setShooterVelocity();
                    //sleep(2000);

                    intakeRampShooter();
                    sleep(6000);

                    //park at white line
                    forwardDistance(0.4, 27);

                    sidewaysDistanceLeft(0.5, 80);
                }

                break;
            }

        }
    }