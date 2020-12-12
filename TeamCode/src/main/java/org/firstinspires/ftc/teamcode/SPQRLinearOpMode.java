package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Date;

/**
 * Custom Linear OpMode class with extra functions. Used mainly for autonomous.
 *
 * @author Arkin Solomon
 * @author Owen Peterson
 */
public abstract class SPQRLinearOpMode extends LinearOpMode {

    //Variables
    public final double ppr = 280;

    //Values for the wheels
    private final double wheelRadius = 5*25.4;
    private final double wheelCircumference = wheelRadius * 2 * Math.PI;

    private int leftFrontEncoder;
    private int rightFrontEncoder;
    private int leftBackEncoder;
    private int rightBackEncoder;

    //Instance of robot hardware class
    public HardwareSPQR robot = new HardwareSPQR(telemetry, true, true);

    /**
     * This method calculates the approximate distance that the robot has traveled with the average
     * encoder value of all of the robot's drive motor's encoders.
     *
     * @return The approximate distance in centimeters (or millimeters, unsure) that the robot has
     * traveled since the last time the encoders were reset to the zero position.
     */
    public double calculateDistance(){
        double encoder = this.driveAverage();
        return (encoder / ppr) * wheelCircumference;
    }

    /**
     * This method takes a value and determines the two values equidistant from the given value with
     * both having a distance of a given value.
     *
     * @param value An integer to be used to calculate final points based on change.
     * @param change An integer which is the absolute distance from the value.
     * @return An array of integers with two indexes with index zero being the smallest possible
     * integer from the given value with a given distance and with index one being the largest
     * possible integer from the given value with a given distance.
     */
    private int[] plusOrMinus(int value, int change) {
        change = Math.abs(change);
        return new int[]{value - change, value + change};
    }

    /**
     * This method drives the robot forward by a given distance in centimeters (or millimeters,
     * unsure) at a specified speed. The robot will go backwards if the speed given is a value less
     * than zero.
     *
     * @param distance A double which represents the distance for the robot to travel in centimeters
     *                 (or millimeters, unsure)
     * @param speed A double between -1.0 and 1.0 which is the speed at which the robot is to drive.
     */
    public void strafe (Dir direction, double distance, double speed){
        DcMotor.ZeroPowerBehavior previousBehavior = this.robot.leftFrontDrive.getZeroPowerBehavior();
        this.robot.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int encoderTarget = (int) ((distance/wheelCircumference)*ppr);
        this.robot.leftFrontDrive.setTargetPosition((direction == Dir.LEFT) ? encoderTarget : -encoderTarget);
        this.robot.leftBackDrive.setTargetPosition((direction == Dir.LEFT) ? -encoderTarget: encoderTarget);
        this.robot.rightFrontDrive.setTargetPosition((direction == Dir.LEFT) ? -encoderTarget: encoderTarget);
        this.robot.rightBackDrive.setTargetPosition((direction == Dir.LEFT) ? encoderTarget : -encoderTarget);
        resetEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.strafe(direction, speed);
        while(drivesBusy() && !isStopRequested() && opModeIsActive()){
            updateTelemetry();
        }
        this.robot.setDriveZeroPowerBehavior(previousBehavior);
        return;
    }
    public void drive(double distance, double speed){
        DcMotor.ZeroPowerBehavior previousBehavior = this.robot.leftFrontDrive.getZeroPowerBehavior();
        this.robot.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int encoderTarget = (int) ((distance/wheelCircumference)*ppr);
        this.robot.setDriveTargetPosition(-encoderTarget);
        resetEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.setPowers(speed);
        while(drivesBusy() && !isStopRequested() && opModeIsActive()){
            updateTelemetry();
        }
        this.robot.setDriveZeroPowerBehavior(previousBehavior);
        return;
    }

    /**
     * This method resets the encoder positions of the drive motors to zero and adds the current
     * encoder position to the total encoder positions.
     */
    public void resetEncoders(DcMotor.RunMode runMode){
        this.leftFrontEncoder += this.robot.leftFrontDrive.getCurrentPosition();
        this.rightFrontEncoder += this.robot.rightFrontDrive.getCurrentPosition();
        this.leftBackEncoder += this.robot.leftBackDrive.getCurrentPosition();
        this.rightBackEncoder += this.robot.rightBackDrive.getCurrentPosition();

        this.robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.setDriveMode(runMode);
    }

    /**
     * This method calculates and returns the average encoder value of all of the robot's drive
     * motors using the values of each encoder position of each drive motor of the robot.
     *
     * @return A double which represents the average encoder values of all of the robot's drive motors.
     */
    public double driveAverage(){
        int[] encoderPositions = {this.robot.leftFrontDrive.getCurrentPosition(), this.robot.rightFrontDrive.getCurrentPosition(), this.robot.leftBackDrive.getCurrentPosition(), this.robot.rightBackDrive.getCurrentPosition()};
        int sum = 0;
        for (int encoderPosition : encoderPositions){
            sum += encoderPosition;
        }
        return sum / encoderPositions.length;
    }

    /**
     * This method updates the telemetry data on both the driver station and the robot controller
     * with common debugging information.
     */
    public void updateTelemetry(){
        telemetry.addData("Distance", calculateDistance());

        telemetry.addData("Left Front Velocity", ((DcMotorEx) this.robot.leftFrontDrive).getVelocity());
        telemetry.addData("Right Front Velocity", ((DcMotorEx) this.robot.rightFrontDrive).getVelocity());
        telemetry.addData("Left Back Velocity", ((DcMotorEx) this.robot.leftBackDrive).getVelocity());
        telemetry.addData("Right Back Velocity", ((DcMotorEx) this.robot.leftFrontDrive).getVelocity());

        telemetry.addData("Left Front Target", this.robot.leftFrontDrive.getTargetPosition());
        telemetry.addData("Right Front Target", this.robot.rightFrontDrive.getTargetPosition());
        telemetry.addData("Left Back Target", this.robot.leftBackDrive.getTargetPosition());
        telemetry.addData("Right Back Target", this.robot.rightBackDrive.getTargetPosition());

        telemetry.addData("left Front Encoder", this.leftFrontEncoder);
        telemetry.addData("right Front Encoder", this.rightFrontEncoder);
        telemetry.addData("left Back Encoder", this.leftBackEncoder);
        telemetry.addData("right Back Encoder", this.rightBackEncoder);
        telemetry.update();
    }

    /**
     *  This method is an abstraction that returns true if at least one motor is running, false if
     *  no motors are running.
     *
     * @return returns the boolean true if one or more drives are running, false otherwise.
     */
    public boolean drivesBusy() {
        return (this.robot.leftFrontDrive.isBusy() || this.robot.rightFrontDrive.isBusy() || this.robot.leftBackDrive.isBusy() || this.robot.rightBackDrive.isBusy());
    }

    /**
     * This method attempts to detect the amount of rings in a stack. If it can not find rings in
     * the specified amount of time, it assumes there are no rings.
     *
     * @param waitFor How long to wait for in milliseconds.
     * @return The detected amount of rings.
     */
    public Rings waitForRings(int waitFor) {
        long startTime = new Date().getTime();
        while (((int) (new Date().getTime() - startTime)) < waitFor){
            Rings update = robot.updateObjectDetection();
            if (update != Rings.NONE){
                return update;
            }
        }
        robot.tfod.deactivate();
        robot.tfod.shutdown();
        return Rings.NONE;
    }
}