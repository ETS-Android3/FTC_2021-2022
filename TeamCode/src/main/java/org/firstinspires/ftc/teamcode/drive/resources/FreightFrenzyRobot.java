package org.firstinspires.ftc.teamcode.drive.resources;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;

/**
 * A class that contains all components and methods
 * of a FTC 2021 – 2022 Freight Frenzy Season robot.
 *
 * @see SampleMecanumDrive
 * @see HardwareMap
 * @see Trajectory
 */
@SuppressWarnings("ConstantConditions")
public class FreightFrenzyRobot {
    // Data field.
    private final HardwareMap hardwareMap;
    public final SampleMecanumDrive drive;
    public Trajectory trajectory;
    private final Timer timer;

    public TouchSensor magneticLightSwitch;
    public HashMap<String, DcMotor> motors;
    public DistanceSensor proximitySensor;
    public HashMap<String, Servo> servos;
    public WebcamName webcam;

    // Setup methods.
    /**
     * Constructor.
     * @param hardwareMap the robot's hardware map.
     */
    public FreightFrenzyRobot(HardwareMap hardwareMap) {
        // Register hardware map and sample mecanum drive.
        this.hardwareMap = hardwareMap;
        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize webcam.
        webcam = hardwareMap.get(WebcamName.class, "Frontcam");

        // Initialize the timer.
        timer = new Timer();

        // Initialize all motors.
        motors = new HashMap<String, DcMotor>() {{
            // Drivetrain motors.
            put("fl",           hardwareMap.dcMotor.get("fl"));
            put("bl",           hardwareMap.dcMotor.get("bl"));
            put("fr",           hardwareMap.dcMotor.get("fr"));
            put("br",           hardwareMap.dcMotor.get("br"));

            // Utility motors.
            put("intake",       hardwareMap.dcMotor.get("intake"));
            put("slides",       hardwareMap.dcMotor.get("slides"));
            // put("carrousel1",   hardwareMap.dcMotor.get("carrousel1"));
            // put("carrousel2",   hardwareMap.dcMotor.get("carrousel2"));
        }};

        /* Initialize all servos.
        servos = new HashMap<String, Servo>() {{
            put("carriage",     hardwareMap.servo.get("carriage"));
            put("intake1",      hardwareMap.servo.get("intake1"));
            put("intake2",      hardwareMap.servo.get("intake2"));
        }};*/

        // Set modes for each motor.
        motors.get("intake").setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the color sensor.
        // proximitySensor = hardwareMap.get(DistanceSensor.class, "proximitySensor");
        // magneticLightSwitch = hardwareMap.get(TouchSensor.class, "magnetLightSwitch");
    }

    // High-level methods.
    /**
     * Input a shipping element.
     */
    public void input(boolean driveForward) {
        dropIntake();
        intake(driveForward);
        while (true)
            if (proximitySensor.getDistance(DistanceUnit.CM) <= Constants.targetProximity) {
                drive.cancelFollowing();
                break;
            }
        stopIntake();
        liftIntake();
        outtake();
        stopIntake();
    }

    /**
     * Outputs a shipping element.
     *
     * @param goal The goal level.
     */
    public void output(String goal) {
        extendSlides(goal);
        emptyCarriage();
        retractSlides();
    }

    /**
     * Spins the appropriate carrousel to drop a duck.
     */
    public void spinCarrousel(String alliance) {
        switch (alliance) {
            case "blue":
                spinBlueCarrousel();
            case "red":
                spinRedCarrousel();
        }
    }

    // Low-level methods.
    /**
     * Intake shipping elements.
     */
    public void intake(boolean driveForward) {
        // Rotate the carriage to the "lowered" position.
        servos.get("carriage").setPosition(Constants.CARRIAGE_POSITIONS[0]);

        // Collect shipping elements.
        trajectory = drive.trajectoryBuilder(trajectory.end())
                .forward(Constants.INTAKE_DISTANCE)
                .build();

        if (driveForward) drive.followTrajectory(trajectory);

        // Turn on the intake.
        motors.get("intake").setPower(Constants.INTAKE_POWERS[0]);
    }

    /**
     * Deactivate intake mechanism.
     */
    public void stopIntake() {
        // Turn off the intake.
        motors.get("intake").setPower(0);

        // Rotate the carriage to the "raised" position.
        servos.get("carriage").setPosition(Constants.CARRIAGE_POSITIONS[1]);
    }

    /**
     * Lift the intake mechanism.
     */
    public void liftIntake() {
        // Raise the intake bar.
        servos.get("intake1").setPosition(Constants.INTAKE_POSITIONS[0]);
        servos.get("intake2").setPosition(Constants.INTAKE_POSITIONS[0]);
    }

    /**
     * Outtake shipping elements.
     */
    public void outtake() {
        // Reverse the intake.
        motors.get("intake").setPower(Constants.INTAKE_POWERS[1]);
    }

    /**
     * Drop the intake mechanism.
     */
    public void dropIntake() {
        // Lower the intake bar.
        servos.get("intake1").setPosition(Constants.INTAKE_POSITIONS[1]);
        servos.get("intake2").setPosition(Constants.INTAKE_POSITIONS[1]);
    }

    /**
     * Extend slides mechanism outward.
     *
     * @param goal The goal level.
     */
    public void extendSlides(String goal) {
        // Set the slides motor's trajectory.
        motors.get("slides").setTargetPosition(Constants.SLIDE_POSITIONS.get(goal));

        // Set power to the slides motor.
        motors.get("slides").setPower(Constants.SLIDES_POWERS[0]);

        // Set the slides motor's mode.
        motors.get("slides").setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Empty the carriage mechanism and restore its position.
     */
    public void emptyCarriage() {
        // Rotate the carriage to the "open" position.
        servos.get("carriage").setPosition(Constants.CARRIAGE_POSITIONS[2]);

        // Delay for a certain amount of time.
        timer.start();
        while (true)
            if (timer.getTime() > Constants.AUTO_TIMES.get("carriage")) {
                timer.stop();
                break;
            }

        // Rotate the carriage to the "raised" position.
        servos.get("carriage").setPosition(Constants.CARRIAGE_POSITIONS[1]);
    }

    /**
     * Retract slides mechanism inward.
     *
     */
    public void retractSlides() {
        // Retract slides motor.
        motors.get("slides").setTargetPosition(0);
        motors.get("slides").setPower(Constants.SLIDES_POWERS[1]);
        motors.get("slides").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Check if magnetic limit switch triggers.
        while (true)
            if (magneticLightSwitch.isPressed())
                break;

        // Reset encoder when magnetic limit switch triggers.
        motors.get("slides").setPower(0);
        motors.get("slides").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Rotate the carriage to the "lowered" position.
        servos.get("carriage").setPosition(Constants.CARRIAGE_POSITIONS[1]);
    }

    /**
     * Spins the blue-side carrousel to drop a duck.
     */
    public void spinBlueCarrousel() {
        // Move blue carrousel motor #1.
        motors.get("carrousel1").setTargetPosition(Constants.CARROUSEL_POSITIONS);
        motors.get("carrousel1").setPower(Constants.CARROUSEL_POWERS);
        motors.get("carrousel1").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset encoder.
        motors.get("carrousel1").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Spins the red-side carrousel to drop a duck.
     */
    public void spinRedCarrousel() {
        // Move red carrousel motor.
        motors.get("carrousel2").setTargetPosition(Constants.CARROUSEL_POSITIONS);
        motors.get("carrousel1").setPower(Constants.CARROUSEL_POWERS);
        motors.get("carrousel2").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset encoder and run to position.
        motors.get("carrousel2").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Getter methods.
    /**
     * Getter method.
     * @return the robot's sample mecanum drive.
     */
    public SampleMecanumDrive getDrive() {
        return drive;
    }

    /**
     * Getter method.
     * @return the robot's hardware map.
     */
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }
}