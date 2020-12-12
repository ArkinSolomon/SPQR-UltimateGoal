package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

/**
 * Hardware configurations
 *
 * Motor channel H1-0 (NeveRest 40 Gearmotor):        Right front drive motor:   "right_front_drive"
 * Motor channel H1-1 (NeveRest 40 Gearmotor):        Left front drive motor:    "left_front_drive"
 * Motor channel H1-2 (NeveRest 40 Gearmotor):        Right back drive motor:    "right_back_drive"
 * Motor channel H1-3 (NeveRest 40 Gearmotor):        Left back drive motor:     "left_back_drive"
 * Motor channel H2-0 (REV Core Hex Motor):           Lower Intake motor:        "low_intake_motor"
 * Servo channel H1-0 (Servo):                        Left intake drop servo:    "l_intake_drop"
 * I2C H1-0-0 (REV Expansion Hub IMU):                Hub connector:            "imu"
 * I2C H2-0-0 (REV Expansion Hub IMU):                Hub connector:            "imu 1"
 */

/**
 * This class defines all hardware on the robot. It also contains common abstractions to interact
 * with hardware.
 *
 * @author Arkin Solomon
 */
public class HardwareSPQR {

    //Vuforia developer key
    private static final String vuforiaKey = "ATssCgr/////AAABmen/Y0Ij5Eo4g95+2iw/E9AdGtmFswuKEHlcF66eyInTs5FCLLEzEzKNyJNpnAdDiOBeQhWH6ftGrQjp/7kYSae5lTqXdFte5FsSFV/MxOY5zvIYGwA/ahozNrG8SDGwn8m7puab/XGbFla4iA2xBZCbZOWAL1GlQONJdX09u8iYAEuxlpJZx2SLpIJtDSwHgR0RaGQGoAL+wbsnanXnAk+wPk9QxgXTOrRNFMbFNqsndWyRro5UeGfTpiRqBDA1na1024KNQd6vfOaDDj6vcX6NiUYYtu06Kd42V5tK4u17c5qFZ6qgki8L33oLi0m3f2RJGV8z/idK5lcD9+JwE4VEveakOLfNij/Yeooot+oO";

    //The name of the trained model
    private static final String modelFileName = "UltimateGoal.tflite";

    //The names of the objects stored in the model
    private static final String object1 = "f";
    private static final String object2 = "o";

    //How many millimeters in an inch
    public float mmPerInch = 25.4f;

    //The width of the robot in millimeters
    public float mmBotWidth = 17.8125f * mmPerInch;

    //The width of the field in millimeters (12 * 12) for 12 1 foot (12 inch) tiles, minus 2 inches for edge connections (1 inch for each edge)
    public float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;

    //Hardware map
    private HardwareMap hwMap = null;

    //Declare hardware
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor lowIntakeMotor = null;
    public Servo lIntakeDrop = null;

    //Whether or not to initialize ring detection
    private boolean initializeRingDetection = false;

    //Whether or not TFOD has been activated
    private boolean isTfodActive = false;

    //Whether or not to initialize vuforia trackables
    private boolean initializeVuforia = false;

    //This stores the vuforia trackables object contaiining all of the vuforia trackables
    private VuforiaTrackables ultimateGoalTrackables;

    //The navigation targets
    private VuforiaTrackable blueTowerGoal, redTowerGoal, redAlliance, blueAlliance, frontWall;

    //Where the phone is located on
    public OpenGLMatrix phoneLocationOnRobot;

    //Vuforia localization engine instance
    public VuforiaLocalizer vuforia = null;

    //The instance of the tensorflow object detector we're using for ring detection
    public TFObjectDetector tfod = null;

    //List of all navigation targets (for convinience)
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    //Id of the camera view (whatever that means)
    public int cameraMonitorViewId = -1;

    //The current location of the robot
    public OpenGLMatrix location = null;

    //True if robot is initialized
    private boolean robotIsInitialized = false;

    //The telemetry (output) instance of the robot
    private Telemetry telemetry = null;

    //If the pacman noise is playing
    private boolean isWakkaPlaying = false;

    //Id of sounds
    private int pacmanChomp = -1;
    private int pacmanStart = -1;

    //Sound player instance
    public SoundPlayer soundPlayer = SoundPlayer.getInstance();

    /**
     * Class constructor, returns a new instance of HardwareSPQR (without ring detection).
     *
     * @param telemetry The telemetry (output) instance of the robot, provided by the OpMode's
     *                  superclass.
     */
    public HardwareSPQR(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Class constructor, returns a new instance of HardwareSPQR (with or without ring detection).
     *
     * @param telemetry The telemetry (output) instance of the robot, provided by the OpMode's
     *                  superclass.
     * @param iV Whether or not to initialize vuforia.
     * @param iRD Whether or not to initialize ring detection.
     */
    public HardwareSPQR(Telemetry telemetry, boolean iV, boolean iRD) {
        this.telemetry = telemetry;
        this.initializeVuforia = iV;
        this.initializeRingDetection = iRD;
        if (iRD && !iV){
            this.initializeVuforia = true;
        }
    }

    /**
     * This method initializes the hardware on the robot including sensors, servos, and motors.
     * This method should be updated whenever a hardware device is added or removed, or when a
     * device's settings need to be modified. This method should be called once.
     *
     * @param m The hardware map of the class instance of the callee to be assigned to the
     *               instance of hardware.
     */
    public void init(HardwareMap m) {

        //Initialize hardware map
        hwMap = m;

        /* Initialize motors*/

        //Define motors
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");
        leftBackDrive = hwMap.get(DcMotor.class, "left_back_drive");
        lowIntakeMotor = hwMap.get(DcMotor.class, "low_intake_motor");

        //Reset encoders and set initial positions
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set drive motors to brake
        setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set encoder tolerance of the motors on the robot
        ((DcMotorEx) rightFrontDrive).setTargetPositionTolerance(10);
        ((DcMotorEx) leftFrontDrive).setTargetPositionTolerance(10);
        ((DcMotorEx) rightBackDrive).setTargetPositionTolerance(10);
        ((DcMotorEx) leftBackDrive).setTargetPositionTolerance(10);

        //Set motor direction
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        lowIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set all motor power to zero
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        lowIntakeMotor.setPower(0);

        /* Servo setup */

        //Define servos
        lIntakeDrop = hwMap.get(Servo.class, "l_intake_drop");

        //Set initial servo position
        lIntakeDrop.setPosition(0);

        /* Sound setup */

        //Get audio id
        pacmanChomp = hwMap.appContext.getResources().getIdentifier("pacman_chomp", "raw", hwMap.appContext.getPackageName());
        pacmanStart = hwMap.appContext.getResources().getIdentifier("pacman_start", "raw", hwMap.appContext.getPackageName());

        /* Vuforia setup */

        if (initializeVuforia){

            //Create parameters
            cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            //Setup parameters
            parameters.vuforiaLicenseKey = vuforiaKey;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //Launch vuforia instance
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            //Load vuforia trackables
            ultimateGoalTrackables = vuforia.loadTrackablesFromAsset("UltimateGoal");
            allTrackables.addAll(ultimateGoalTrackables);
            blueTowerGoal = ultimateGoalTrackables.get(0);
            blueTowerGoal.setName("BlueTowerGoal");
            redTowerGoal = ultimateGoalTrackables.get(1);
            redTowerGoal.setName("RedTowerGoal");
            redAlliance = ultimateGoalTrackables.get(2);
            redAlliance.setName("RedAlliance");
            blueAlliance = ultimateGoalTrackables.get(3);
            blueAlliance.setName("BlueAlliance");
            frontWall = ultimateGoalTrackables.get(4);
            frontWall.setName("FrontWall");

            //Set target locations
            OpenGLMatrix blueTowerGoalLocation = OpenGLMatrix
                    .translation(mmFTCFieldWidth / 2, mmFTCFieldWidth / 4, 5.75f * mmPerInch)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ,
                            AngleUnit.DEGREES, 90, 0, -90));
            blueTowerGoal.setLocation(blueTowerGoalLocation);
            RobotLog.ii("Vuforia Navigation", "Blue tower goal location=" + blueTowerGoalLocation.toString());

            OpenGLMatrix redTowerGoalLocation = OpenGLMatrix
                    .translation(mmFTCFieldWidth / 2, -mmFTCFieldWidth / 4, 5.75f * mmPerInch)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ,
                            AngleUnit.DEGREES, 90, 0, -90));
            redTowerGoal.setLocation(blueTowerGoalLocation);
            RobotLog.ii("Vuforia Navigation", "Red tower goal location=" + redTowerGoalLocation.toString());

            OpenGLMatrix redAllianceLocation = OpenGLMatrix
                    .translation(0, -mmFTCFieldWidth / 2, 5.75f * mmPerInch)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ,
                            AngleUnit.DEGREES, 90, 0, 180));
            redAlliance.setLocation(redAllianceLocation);
            RobotLog.ii("Vuforia Navigation", "Red alliance location=" + redAllianceLocation.toString());

            OpenGLMatrix blueAllianceLocation = OpenGLMatrix
                    .translation(0, mmFTCFieldWidth / 2, 5.75f * mmPerInch)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ,
                            AngleUnit.DEGREES, 90, 0, 0));
            blueAlliance.setLocation(blueAllianceLocation);
            RobotLog.ii("Vuforia Navigation", "Blue alliance location=" + blueAllianceLocation.toString());

            OpenGLMatrix frontWallLocation = OpenGLMatrix
                    .translation(0, -mmFTCFieldWidth / 2, 5.75f * mmPerInch)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ,
                            AngleUnit.DEGREES, 90, 0, 90));
            frontWall.setLocation(redAllianceLocation);
            RobotLog.ii("Vuforia Navigation", "Front wall location=" + frontWallLocation.toString());

            //Set phone location
            phoneLocationOnRobot = OpenGLMatrix
                    .translation(mmBotWidth/2,0,0)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ,
                            AngleUnit.DEGREES, 90, 90, 0));
            RobotLog.ii("Vuforia Navigation", "Phone location=" + phoneLocationOnRobot.toString());
            ((VuforiaTrackableDefaultListener) blueTowerGoal.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) redTowerGoal.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) redAlliance.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) blueAlliance.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) frontWall.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

            //Activate all trackables
            ultimateGoalTrackables.activate();
        }

        /* Initialize tensor flow object detection */

        //Initialize instance
        if (initializeRingDetection) {
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(hwMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hwMap.appContext.getPackageName()));
            tfodParameters.minResultConfidence = 0.3f;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(modelFileName, object1, object2);
        }

        this.robotIsInitialized = true;

        //Start sound
        soundPlayer.startPlaying(hwMap.appContext, pacmanStart);
    }

    /**
     * This method sets the drive behavior of what one of the drive motors of the robot are to do
     * when its power is set to 0.
     *
     * @param behavior A ZeroPowerBehavior enumeration (Under DcMotor) which will be applied to all
     *                 of the drive motors of the robot.
     */
    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    /**
     * This method sets the drive mode of all of the drive motors.
     *
     * @param mode a RunMode enumeration (Under DcMotor) which will be applied to all
     *             of the drive motors of the robot.
     */
    public void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    /**
     * Raise the lower intake.
     */
    public void raiseIntake() {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        lIntakeDrop.setPosition(0);
    }

    /**
     * Drop the lower intake.
     */
    public void dropIntake() {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        lIntakeDrop.setPosition(-1);
    }

    /**
     * This method sets the powers of the motors in a way that will strafe the robot in a given
     * direction.
     *
     * @param direction A Dir enumeration which states which direction the robot will strafe.
     * @param power     A double between -1.0 and 1.0 which represents the speed at which the robot
     *                  is to strafe.
     */
    public void strafe(Dir direction, double power) {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        leftFrontDrive.setPower((direction == Dir.LEFT) ? -power : power);
        leftBackDrive.setPower((direction == Dir.LEFT) ? power : -power);
        rightFrontDrive.setPower((direction == Dir.LEFT) ? power : -power);
        rightBackDrive.setPower((direction == Dir.LEFT) ? -power : power);
        if (power != 0){
//            soundPlayer.startPlaying(hwMap.appContext, pacmanChomp);
        }else{
            soundPlayer.stopPlayingAll();
        }
    }

    /**
     * This method sets all of the motors to specific given power.
     *
     * @param power A double between -1.0 and 1.0 which represents the speed that the motors will
     *              be set to.
     */
    public void setPowers(double power) {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        if (power != 0){
//            soundPlayer.startPlaying(hwMap.appContext, pacmanChomp);
        }else{
            soundPlayer.stopPlayingAll();
        }
    }

    /**
     * This method makes the robot go at full speed backward by setting all of the motor powers to
     * -1.0.
     */
    public void backward() {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        this.setPowers(-1.0);
//        soundPlayer.startPlaying(hwMap.appContext, pacmanChomp);
    }

    /**
     * This method makes the robot go at full speed forward by setting all of the motor powers to
     * 1.0.
     */
    public void forward() {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        this.setPowers(1.0);
//        soundPlayer.startPlaying(hwMap.appContext, pacmanChomp);
    }

    /**
     * This method takes two powers and sets the left motors to a given power and a right motors to
     * a given power respectively.
     *
     * @param left  A double between -1.0 and 1.0 which determines the speed that the left motors
     *              will be set to.
     * @param right A double between -1.0 and 1.0 which determines the speed that the right motors
     *              will be set to.
     */
    public void tank(double left, double right) {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        leftFrontDrive.setPower(left);
        leftBackDrive.setPower(left);
        rightFrontDrive.setPower(right);
        rightBackDrive.setPower(right);
        if (left != 0 || right != 0){
//            soundPlayer.startPlaying(hwMap.appContext, pacmanChomp);
        }else if (left == 0 && right == 0){
            soundPlayer.stopPlayingAll();
        }
    }

    /**
     * This method stops the movement of the drive motors. If the motors are set to brake on zero
     * power the robot will stop in place.
     */
    public void stopMoving() {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        this.setPowers(0);
        soundPlayer.stopPlayingAll();
    }

    /**
     * This method sets the same target position to all of the drive motors of the robot. The motors
     * still need to be given power to move to the position.
     *
     * @param target An integer which is the target, in encoder units, for the robot to go to.
     */
    public void setDriveTargetPosition(int target) {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        this.leftFrontDrive.setTargetPosition(target);
        this.rightFrontDrive.setTargetPosition(target);
        this.leftBackDrive.setTargetPosition(target);
        this.rightBackDrive.setTargetPosition(target);
    }

    /**
     * This method updates the position of the robot according to the detected navigation targets.
     * The updated location can be accessed through the location variable.
     */
    public void updateRobotPosition() {
        if (!this.robotIsInitialized || !this.initializeVuforia)throw new Error("Robot not initialized");;
        for (VuforiaTrackable trackable : allTrackables) {
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                location = robotLocationTransform;
            }
        }

        //Provide feedback as to where the robot was last located (if we know).
        if (location != null) {
            telemetry.addData("Pos", location.formatAsTransform());
        } else {
            telemetry.addData("Pos", "Unknown");
        }
    }

    /**
     * Get how many rings the robot detects in the starter stack.
     *
     * @return The number of rings in the starter stack from an enumeration.
     */
    public Rings updateObjectDetection() {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        if (initializeRingDetection){
            if (!isTfodActive){
                tfod.activate();
                isTfodActive = true;
            }
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size() > 0) {
                String firstRecognition = updatedRecognitions.get(0).getLabel();
                return firstRecognition == "f" ? Rings.FOUR : Rings.ONE;
            }
        }
        return Rings.NONE;
    }
}