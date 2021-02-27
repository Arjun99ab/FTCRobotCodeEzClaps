package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BlueLeft", group="Linear Opmode")
//@Disabled
public class BlueLeft extends AutonomousController {

    double firstDegrees;

    @Override
    public void runOpMode(){

        initRobot();
        applyBrake();

        //wobblePos = "A";


        if(opModeIsActive()) {
            if(wobblePos == "A") {

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

                forwardDistance(0.7, 190);
                //forwardDistance(0.3, 20);

                resetEncoders();

                GyroTurn(170, 0.6);

                //lower
                powerCRServo2(-0.8);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                forwardDistance(0.4, 37);

                resetEncoders();

                GyroTurn(-171, 0.4);

                //navigate to shooter position

                sidewaysDistanceRight(0.3, 33);

                //setShooterVelocity();
                //sleep(2000);

                intakeRampShooter();
                sleep(6000);

                //park at white line
                forwardDistance(0.4, 27);

                sidewaysDistanceLeft(0.5, 50);

            } else if(wobblePos == "B") {

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

                forwardDistance(0.9, 246);

                forward(0);
                sleep(100);

                //lower
                powerCRServo2(-0.8);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                //navigate to shooter position
                backwardDistance(0.5, 90); //idk distances
                sidewaysDistanceRight(0.45, 26); //idk distances

                intakeRampShooter();
                sleep(5000);

                //park at white line
                forwardDistance(0.4, 27);

                sidewaysDistanceLeft(0.5, 50);

            } else if(wobblePos == "C") {
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

                sidewaysDistanceLeft(0.3, 32);

                forwardDistance(0.8, 302);

                sidewaysDistanceRight(0.4, 33);

                resetEncoders();

                forward(0);
                sleep(100);

                GyroTurn(-170, 0.5);

                //lower
                powerCRServo2(-0.8);
                sleep(1000);

                //ungrip
                powerCRServo(-0.6);
                sleep(1000);

                GyroTurn(170, 0.5);

                //navigate to shooter position
                backwardDistance(0.6, 148);

                sidewaysDistanceRight(0.3, 20);

                intakeRampShooter();
                sleep(5000);

                //park at white line
                forwardDistance(0.4, 27);

                sidewaysDistanceLeft(0.5, 40);

            } else {
                forward(0);
            }

        }

    }
}
