package org.firstinspires.ftc.teamcode.drive.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.resources.Constants;
import org.firstinspires.ftc.teamcode.drive.resources.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.drive.resources.Recognizer;
import org.firstinspires.ftc.teamcode.drive.resources.Timer;

import java.util.HashMap;

/**
 * The 2021 – 2022 FTC #5421 autonomous algorithm (on the blue side).
 * @see FreightFrenzyRobot
 */
@SuppressWarnings("ConstantConditions")
@Autonomous(name="Blue Freight Frenzy Autonomous")
public class BlueAuto extends LinearOpMode {
    // Data field.
    private FreightFrenzyRobot robot;
    private SampleMecanumDrive drive;
    private Trajectory trajectory;
    private Recognizer recognizer;
    private Timer timer;

    // Constants.
    private final Pose2d start = new Pose2d(-36, 60, 180);
    private final HashMap<String, Vector2d> locations = new HashMap<String, Vector2d>() {{
        put("carrousel", new Vector2d(-60, 60));
        put("fountain1", new Vector2d(-24, 36));
        put("passage",   new Vector2d(24, 60));
        put("warehouse", new Vector2d(58, 60));
        put("fountain2", new Vector2d(58, 12));
        put("parking",   new Vector2d(-60, 36));
    }};
    private final HashMap<String, Double> tangents = new HashMap<String, Double>() {{
        put("carrousel", Math.toRadians(180));
        put("fountain1", Math.toRadians(-45));
        put("passage",   Math.toRadians(0));
        put("warehouse", Math.toRadians(0));
        put("fountain2", Math.toRadians(225));
        put("parking",   Math.toRadians(270));
    }};

    /**
     * Run method.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("1", "Hello");
        telemetry.update();

        // Initialize robot.
        initialize();

        // Wait for controller to start the program.
        waitForStart();

        // FSM for autonomous algorithm.
        if (opModeIsActive() && !isStopRequested()) {
            // Start the timer.
            timer.start();

            // Set the initial position estimate.
            drive.setPoseEstimate(start);

            // Declare parameters for image recognition.
            String goal;

            // Switch statement for image recognition.
            switch (recognizer.getAnalysis()[0]) {
                case LEFT:
                    goal = "low";
                case CENTER:
                    goal = "middle";
                case RIGHT:
                    goal = "high";
                default:
                    goal = "low";
            }

            // Drive to the duckies :)
            trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(locations.get("carrousel"), tangents.get("carrousel"))
                    .build();
            drive.followTrajectory(trajectory);

            // Run the carrousel to spin the ducky.
            // robot.spinCarrousel("blue");

            // Turn the robot.
            drive.turn(-90);

            // Spline to the start fountain.
            trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(locations.get("fountain1"), tangents.get("fountain1"))
                    .build();
            drive.followTrajectory(trajectory);

            // Output a shipping element.
            // robot.output(goal);

            // Turn the robot.
            drive.turn(90);

            // Enter the warehouse.
            trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(locations.get("passage"), tangents.get("passage"))
                    .build();
            drive.followTrajectory(trajectory);

            // Transport cycles.
            while (timer.getTime() < Constants.AUTO_TIMES.get("cycles")) {
                // Drive to the warehouse.
                trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(locations.get("warehouse"), tangents.get("warehouse"))
                        .build();
                drive.followTrajectory(trajectory);

                // Input a shipping element.
                // robot.intake(true);

                // Turn the robot.
                drive.turn(-90);

                // Drive to alliance fountain.
                trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(locations.get("fountain2"), tangents.get("fountain2"))
                        .build();
                drive.followTrajectory(trajectory);

                // Output the shipping element, defaulting to the low goal.
                robot.output("low");
            }

            // Enter the warehouse.
            trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(locations.get("passage"),
                            Math.toRadians(180) - tangents.get("passage"))
                    .build();
            drive.followTrajectory(trajectory);

            // Drive to parking spot.
            trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(locations.get("parking"), tangents.get("parking"))
                    .build();
            drive.followTrajectory(trajectory);
        }
    }

    private void initialize() {
        // Component initialization.
        robot = new FreightFrenzyRobot(hardwareMap);
        trajectory = robot.trajectory;
        drive = robot.drive;

        // Initialize the recognition algorithm.
        recognizer = new Recognizer(telemetry);

        // Initialize the timer.
        timer = new Timer();
    }

    /**
     * Makes the OLD robot go in a circle.
     */
    private void circle() {
        // Define a constant.
        final double constant = 20;

        // Start the circle.
        robot.trajectory = robot.drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(constant, -constant), Math.toRadians(45))
                .splineTo(new Vector2d(constant, constant), Math.toRadians(135))
                .build();
        robot.drive.followTrajectory(robot.trajectory);

        // Perform a small action.
        robot.drive.turn(Math.toRadians(-135));
        robot.drive.turn(Math.toRadians(135));

        // Complete the circle.
        robot.trajectory = robot.drive.trajectoryBuilder(robot.trajectory.end())
                .splineTo(new Vector2d(-constant, constant), Math.toRadians(225))
                .splineTo(new Vector2d(-constant, -constant), Math.toRadians(-45))
                .splineTo(new Vector2d(constant, -constant), Math.toRadians(45))
                .splineTo(new Vector2d(0, 0), Math.toRadians(90))
                .build();
        robot.drive.followTrajectory(robot.trajectory);

        // Turn around to the starting position.
        robot.drive.turn(-90);
    }
}
