package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="TeleOpIntake", group="Linear Opmode")
//@Disabled
public class TeleOpIntake extends TeleOpController {

    @Override
    public void runOpMode()  {
        //Initiate TeleOP
        initRobot();

        shooter.setVelocityPIDFCoefficients(1.37, 0.137, 0, 13.7);
        shooter.setPositionPIDFCoefficients(5.0);

        //charge & move, move & intake
        double left_y1 = 0;
        double left_x1 = 0;

        double right_x1 = 0;

        double left_y2 = 0;
        double right_y2 = 0;

        double shooterVelocity = 1680;
        double intakePower = 0.5;

        double pow = 1;

        double coolpower = 0.53;
        //applyBrake();

        while (opModeIsActive()) {



            if((left_y1 != 0 || left_x1 != 0 || right_x1 != 0) && gamepad2.a){

                topLeft.setPower(left_y1 + left_x1 + right_x1);
                botLeft.setPower(left_y1 - left_x1 + right_x1);
                topRight.setPower(left_y1 - left_x1 - right_x1);
                botRight.setPower(left_y1 + left_x1 - right_x1);
                    intakeMotor.setDirection(CRServo.Direction.FORWARD);
                    intakeMotor.setPower(0.5);

            } else if(gamepad1.dpad_up && gamepad2.a ){
                    forward(pow);
                    intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    intakeMotor.setPower(intakePower);

            } else if(gamepad1.dpad_down && gamepad2.a ) {
                    backward(pow);
                    intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    intakeMotor.setPower(intakePower);

            } else if(gamepad1.dpad_down && gamepad2.y ) {
                    backward(pow);
                    setShooterVelocityInput(shooterVelocity);
                    rampMotor.setPower(1);
                    intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    intakeMotor.setPower(intakePower);



            }else if(gamepad1.dpad_up && gamepad2.y ){

                    forward(pow);
                    setShooterVelocityInput(shooterVelocity);
                    rampMotor.setPower(1);
                    intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    intakeMotor.setPower(intakePower);

            } else if((left_y1 != 0 || left_x1 != 0 || right_x1 != 0) && right_y2 != 0) {

                topLeft.setPower(left_y1 + left_x1 + right_x1);
                botLeft.setPower(left_y1 - left_x1 + right_x1);
                topRight.setPower(left_y1 - left_x1 - right_x1);
                botRight.setPower(left_y1 + left_x1 - right_x1);
                    rampMotor.setPower(right_y2);


            } else if((left_y1 != 0 || left_x1 != 0 || right_x1 != 0) && gamepad2.dpad_up) {

                topLeft.setPower(left_y1 + left_x1 + right_x1);
                botLeft.setPower(left_y1 - left_x1 + right_x1);
                topRight.setPower(left_y1 - left_x1 - right_x1);
                botRight.setPower(left_y1 + left_x1 - right_x1);
                    shooterVelocity = shooterVelocity + 24;
                    telemetry.addData("Velocity", shooterVelocity);
                    telemetry.update();
                    sleep(200);

            } else if(gamepad1.dpad_up && gamepad2.dpad_up) {

                    forward(pow);
                    shooterVelocity = shooterVelocity + 24;
                    telemetry.addData("Velocity", shooterVelocity);
                    telemetry.update();
                    sleep(200);

            }

            else if((left_y1 != 0 || left_x1 != 0 || right_x1 != 0) && gamepad2.dpad_right) {

                topLeft.setPower(left_y1 + left_x1 + right_x1);
                botLeft.setPower(left_y1 - left_x1 + right_x1);
                topRight.setPower(left_y1 - left_x1 - right_x1);
                botRight.setPower(left_y1 + left_x1 - right_x1);
                    shooterVelocity = 1344;
                    telemetry.addData("Velocity", shooterVelocity);
                    telemetry.update();
                    sleep(200);

            } else if(gamepad1.dpad_up && gamepad2.dpad_right) {

                    forward(pow);
                    shooterVelocity = 1344;
                    telemetry.addData("Velocity", shooterVelocity);
                    telemetry.update();
                    sleep(200);

            }

            else if(gamepad2.dpad_up) {
                intakePower = intakePower + 0.02;
                telemetry.addData("Velocity", intakePower);
                telemetry.update();
                sleep(200);
            } else if(gamepad2.dpad_down) {
                intakePower = intakePower - 0.02;
                telemetry.addData("Velocity", intakePower);
                telemetry.update();
                sleep(200);

            } else if(gamepad2.y) {

                rampMotor.setPower(1);
                shooter.setVelocity(shooterVelocity);
                intakeMotor.setDirection(CRServo.Direction.FORWARD);
                intakeMotor.setPower(intakePower);

            } else if(gamepad2.a) {
                intakeMotor.setDirection(CRServo.Direction.FORWARD);
                intakeMotor.setPower(intakePower);
            }
            else if(right_y2 != 0) {
                rampMotor.setPower(right_y2);
            }
            else if(gamepad2.left_stick_y != 0) {
                intakeMotor.setPower(gamepad2.left_stick_y);
            }
            else if(gamepad1.right_bumper) {
                setShooterVelocityInput(shooterVelocity);
            }
            else if(gamepad2.dpad_left) {
                shooterVelocity = 1152;
                telemetry.addData("Velocity", shooterVelocity);
                telemetry.update();
                sleep(200);
            }
            else if(gamepad2.dpad_right) {
                shooterVelocity = 1680;
                telemetry.addData("Velocity", shooterVelocity);
                telemetry.update();
                sleep(200);
            }
            /*
            else if(gamepad1.y) { //for left powershot
                shooterVelocity = 1224;
                currentDistance = distanceSensorLeft.getDistance(DistanceUnit.CM);
                currentDistance2 = distanceSensorFront.getDistance(DistanceUnit.CM);
                moveDistanceSensorPowershotSideways(0.4, 50.5); //
                moveDistanceSensorForward(0.4, 154); //check distance
            }
            else if(gamepad1.x) { //for middle powershot
                shooterVelocity = 1224;
                currentDistance = distanceSensorLeft.getDistance(DistanceUnit.CM);
                currentDistance2 = distanceSensorFront.getDistance(DistanceUnit.CM);
                moveDistanceSensorPowershotSideways(0.4, 73);
                moveDistanceSensorForward(0.4, 154); //check distance
            }
            else if(gamepad1.a) { //for right powershot
                shooterVelocity = 1224;
                currentDistance = distanceSensorLeft.getDistance(DistanceUnit.CM);
                currentDistance2 = distanceSensorFront.getDistance(DistanceUnit.CM);
                moveDistanceSensorPowershotSideways(0.4, 90.5);
                moveDistanceSensorForward(0.4, 154); //check distance
            }

             */


            else if(gamepad2.left_bumper) {
                setShooterVelocityInput(0);
                powerCRServo(0);
                forward(0);
                rampMotor.setPower(0);
                intakeMotor.setPower(0);
            }
            else {
                setShooterVelocityInput(0);
                powerCRServo(0);
                forward(0);
                rampMotor.setPower(0);
                intakeMotor.setPower(0);
            }
        }
    }
}
