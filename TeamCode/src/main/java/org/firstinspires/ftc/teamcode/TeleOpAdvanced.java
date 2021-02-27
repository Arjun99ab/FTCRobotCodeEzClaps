package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//dogtreat
@TeleOp(name="TeleOpAdvanced", group="Linear Opmode")
//@Disabled
public class TeleOpAdvanced extends TeleOpController {

    @Override
    public void runOpMode()  {
        //Initiate TeleOP
        initRobot();

        boolean isInHighSpeedMode = true;
        boolean isUp = false;

        shooter.setVelocityPIDFCoefficients(1.37, 0.137, 0, 13.7);
        shooter.setPositionPIDFCoefficients(5.0);

        //charge & move, move & intake
        double left_y1 = 0;
        double left_x1 = 0;

        double right_x1 = 0;

        double left_y2 = 0;
        double right_y2 = 0;

        double shooterVelocity = 1704;

        double pow = 1;

        double coolpower = 0.53;
        //applyBrake();

        while (opModeIsActive()) {


            if(isInHighSpeedMode){
                left_y2 = this.gamepad2.left_stick_y;
                right_y2 = this.gamepad2.right_stick_y;

                left_y1 = -this.gamepad1.left_stick_y;
                left_x1 = this.gamepad1.left_stick_x;

                right_x1 = this.gamepad1.right_stick_x;

                pow = 1;
            } else if(isInHighSpeedMode == false){
                left_y2 = this.gamepad2.left_stick_y/2;
                right_y2 = this.gamepad2.right_stick_y/2;

                left_y1 = -this.gamepad1.left_stick_y/2;
                left_x1 = this.gamepad1.left_stick_x/2;

                right_x1 = this.gamepad1.right_stick_x/2;

                pow = 1/2;

            }

            topLeft.setPower(left_y1 + left_x1 + right_x1);
            botLeft.setPower(left_y1 - left_x1 + right_x1);
            topRight.setPower(left_y1 - left_x1 - right_x1);
            botRight.setPower(left_y1 + left_x1 - right_x1);

            if((left_y1 != 0 || left_x1 != 0 || right_x1 != 0) && gamepad2.a){

                topLeft.setPower(left_y1 + left_x1 + right_x1);
                botLeft.setPower(left_y1 - left_x1 + right_x1);
                topRight.setPower(left_y1 - left_x1 - right_x1);
                botRight.setPower(left_y1 + left_x1 - right_x1);
                    intakeMotor.setDirection(CRServo.Direction.FORWARD);
                    intakeMotor.setPower(0.65);

            } else if(gamepad1.dpad_up && gamepad2.a ){
                    forward(pow);
                    intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    intakeMotor.setPower(0.65);

            } else if(gamepad1.dpad_down && gamepad2.a ) {
                    backward(pow);
                    intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    intakeMotor.setPower(0.65);

            } else if(gamepad1.dpad_down && gamepad2.y ) {
                    backward(pow);
                    setShooterVelocityInput(shooterVelocity);
                    rampMotor.setPower(1);
                    intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    intakeMotor.setPower(0.65);

            } else if((left_y1 != 0 || left_x1 != 0 || right_x1 != 0) && gamepad1.right_bumper) {
                topLeft.setPower(left_y1 + left_x1 + right_x1);
                botLeft.setPower(left_y1 - left_x1 + right_x1);
                topRight.setPower(left_y1 - left_x1 - right_x1);
                botRight.setPower(left_y1 + left_x1 - right_x1);
                    setShooterVelocityInput(shooterVelocity);

            }
            else if((left_y1 != 0 || left_x1 != 0 || right_x1 != 0) && gamepad2.y){
                topLeft.setPower(left_y1 + left_x1 + right_x1);
                botLeft.setPower(left_y1 - left_x1 + right_x1);
                topRight.setPower(left_y1 - left_x1 - right_x1);
                botRight.setPower(left_y1 + left_x1 - right_x1);
                    setShooterVelocityInput(shooterVelocity);
                    rampMotor.setPower(1);
                    intakeMotor.setDirection(CRServo.Direction.FORWARD);
                    intakeMotor.setPower(0.65);

            }else if(gamepad1.dpad_up && gamepad2.y ){

                    forward(pow);
                    setShooterVelocityInput(shooterVelocity);
                    rampMotor.setPower(1);
                    intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    intakeMotor.setPower(0.65);

            } else if((left_y1 != 0 || left_x1 != 0 || right_x1 != 0) && right_y2 != 0) {

                topLeft.setPower(left_y1 + left_x1 + right_x1);
                botLeft.setPower(left_y1 - left_x1 + right_x1);
                topRight.setPower(left_y1 - left_x1 - right_x1);
                botRight.setPower(left_y1 + left_x1 - right_x1);
                    rampMotor.setPower(right_y2);

            } else if(gamepad1.dpad_up && right_y2 != 0) {

                    forward(pow);
                    rampMotor.setPower(right_y2);

            } else if(gamepad1.dpad_down && right_y2 != 0) {

                    backward(pow);
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
            else if((left_y1 != 0 || left_x1 != 0 || right_x1 != 0) && gamepad2.dpad_down) {

                topLeft.setPower(left_y1 + left_x1 + right_x1);
                botLeft.setPower(left_y1 - left_x1 + right_x1);
                topRight.setPower(left_y1 - left_x1 - right_x1);
                botRight.setPower(left_y1 + left_x1 - right_x1);
                    shooterVelocity = shooterVelocity - 24;
                    telemetry.addData("Velocity", shooterVelocity);
                    telemetry.update();
                    sleep(200);

            } else if(gamepad1.dpad_up && gamepad2.dpad_down) {

                    forward(pow);
                    shooterVelocity = shooterVelocity - 24;
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
            else if((left_y1 != 0 || left_x1 != 0 || right_x1 != 0) && gamepad2.dpad_left) {

                topLeft.setPower(left_y1 + left_x1 + right_x1);
                botLeft.setPower(left_y1 - left_x1 + right_x1);
                topRight.setPower(left_y1 - left_x1 - right_x1);
                botRight.setPower(left_y1 + left_x1 - right_x1);
                    shooterVelocity = 1152;
                    telemetry.addData("Velocity", shooterVelocity);
                    telemetry.update();
                    sleep(200);

            } else if(gamepad1.dpad_up && gamepad2.dpad_left) {

                    forward(pow);
                    shooterVelocity = 1152;
                    telemetry.addData("Velocity", shooterVelocity);
                    telemetry.update();
                    sleep(200);

            }
            else if(gamepad1.dpad_up){
                forward(pow);
            }else if(gamepad1.dpad_down){
                backward(pow);
            } else if(gamepad1.dpad_left){
                sidewaysLeft(pow);
            } else if(gamepad1.dpad_right){
                sidewaysRight(pow);
            }
            else if(gamepad2.dpad_up) {
                shooterVelocity = shooterVelocity + 24;
                telemetry.addData("Velocity", shooterVelocity);
                telemetry.update();
                sleep(200);
            } else if(gamepad2.dpad_down) {
                shooterVelocity = shooterVelocity - 24;
                telemetry.addData("Velocity", shooterVelocity);
                telemetry.update();
                sleep(200);
            }
            else if(gamepad2.x) {
                isUp = false;
                //lower
                powerCRServo2(-0.8);
                sleep(600);

                //ungrip
                powerCRServo(-0.6);
                sleep(300);
            } else if(gamepad2.y) {

                rampMotor.setPower(1);
                shooter.setVelocity(shooterVelocity);
                intakeMotor.setDirection(CRServo.Direction.FORWARD);
                intakeMotor.setPower(0.65);

            } else if(gamepad2.a) {
                intakeMotor.setDirection(CRServo.Direction.FORWARD);
                intakeMotor.setPower(0.65);
            } else if(gamepad2.b && isUp == false) {
                isUp = true;
                //grip
                powerCRServo(1);
                sleep(400);

                //raise
                powerCRServo2(1);
                sleep(1200);
            } else if(gamepad2.right_bumper) {
                //lower
                powerCRServo2(-0.8);
                sleep(600);

                //ungrip
                powerCRServo(-0.6);
                sleep(300);

                //raise
                powerCRServo2(1);
                sleep(1200);
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
                shooterVelocity = 1704;
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

            else if(gamepad1.b) { //for high goal
                shooterVelocity = 1344;
                currentDistance2 = distanceSensorFront.getDistance(DistanceUnit.CM);
                applyBrake();
                moveDistanceSensorForward(0.4, 139); //check distance
                sleep(500);
                applyCoast();
            }

            else if(gamepad2.left_bumper){
                setShooterVelocityInput(0);
                powerCRServo(0);
                forward(0);
                rampMotor.setPower(0);
                intakeMotor.setPower(0);
            } else if(gamepad1.left_bumper && crServo.getPower() != 0 && crServo2.getPower() != 0) {
                if(isInHighSpeedMode == true){
                    isInHighSpeedMode = false;
                } else if(isInHighSpeedMode == false){
                    isInHighSpeedMode = true;
                }
                telemetry.addData("High Speed Mode", isInHighSpeedMode);
            }
            else if(gamepad1.left_bumper){
                if(isInHighSpeedMode == true){
                    isInHighSpeedMode = false;
                } else if(isInHighSpeedMode == false){
                    isInHighSpeedMode = true;
                }
                telemetry.addData("High Speed Mode", isInHighSpeedMode);
            }

            else if(gamepad1.left_bumper && (left_y1 != 0 || left_x1 != 0 || right_x1 != 0)) {
                if(isInHighSpeedMode == true){
                    isInHighSpeedMode = false;
                } else if(isInHighSpeedMode == false){
                    isInHighSpeedMode = true;
                }
                telemetry.addData("High Speed Mode", isInHighSpeedMode);

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
