package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//arjun doent remember asking
@Autonomous(name="RedRightAlliance", group="Linear Opmode")
//@Disabled
public class RedRightAlliance extends AutonomousController {

    double firstDegrees;

    @Override
    public void runOpMode()
    {
        initRobot();


            while(opModeIsActive()) {
                if(wobblePos == "B") {

                    boolean isClamped = false;

                    setShooterVelocity();

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

                    sleep(2000);

                    //navigate to shooter position
                    backwardDistance(0.5, 89.5); //idk distances
                    sidewaysDistanceRight(0.45, 37); //idk distances

                    intakeRampShooter();
                    sleep(5000);

                    //park at white line
                    forwardDistance(0.4, 27);

                    sidewaysDistanceRight(0.5, 50);

                } else if(wobblePos == "C") {
                    boolean isClamped = false;

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

                    sidewaysDistanceRight(0.3, 32);

                    setShooterVelocity();

                    forwardDistance(0.9, 310);

                    forward(0);
                    sleep(100);

                    sidewaysDistanceLeft(0.4, 32);

                    //lower
                    powerCRServo2(-0.8);
                    sleep(1000);

                    //ungrip
                    powerCRServo(-0.6);
                    sleep(1000);

                    //navigate to shooter position
                    backwardDistance(0.6, 155.5);
                    sidewaysDistanceLeft(0.3, 26);

                    intakeRampShooter();
                    sleep(4100);

                    //park at white line
                    forwardDistance(0.4, 27);

                    //sidewaysDistanceRight(0.5, 50);

                } else {
                    boolean isClamped = false;

                    setShooterVelocity();

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
                    backwardDistance(0.4, 35);
                    sidewaysDistanceLeft(0.3, 30.5);

                    //setShooterVelocity();
                    //sleep(2000);

                    intakeRampShooter();
                    sleep(6000);

                    //park at white line
                    forwardDistance(0.4, 27);

                    sidewaysDistanceRight(0.5, 8);
                }

                break;
            }

        }
    }