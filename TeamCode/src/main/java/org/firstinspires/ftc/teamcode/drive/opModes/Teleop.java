package org.firstinspires.ftc.teamcode.drive.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.resources.FreightFrenzyRobot;

/**
 * An FSM implementation of the 2021-2022 FTC #5421 tele-operation algorithm.
 * @see FreightFrenzyRobot
 */
@TeleOp(name="Freight Frenzy Teleop")
public class Teleop extends LinearOpMode {
    // Data field.
    private FreightFrenzyRobot robot;
    private State state;

    /**
     * An enum to represent the robot's lift state.
     */
    public enum State {
        DRIVE,
        INPUT,
        OUTPUT,
        CARROUSEL_LEFT,
        CARROUSEL_RIGHT,
        RESET
    }

    /**
     * Run method.
     */
    @Override
    public void runOpMode() {
        // Component initialization.
        robot = new FreightFrenzyRobot(hardwareMap);

        // Initialize the robot's state.
        state = State.DRIVE;

        // Initialize drive thread.
        Thread drive = new Thread(() -> {
            while (opModeIsActive())
                drive();
        });

        // Initialize drive thread.
        Thread action = new Thread(() -> {
            while (opModeIsActive())
                // Action case.
                switch (state) {
                    case INPUT:
                        robot.input(false);
                        state = State.DRIVE;
                    case OUTPUT:
                        robot.output("high");
                        state = State.DRIVE;
                    case CARROUSEL_LEFT:
                        robot.spinCarrousel("blue");
                        state = State.DRIVE;
                    case CARROUSEL_RIGHT:
                        robot.spinCarrousel("red");
                        state = State.DRIVE;
                }
        });

        // Start threads.
        drive.start();
        action.start();
    }

    /**
     * Drive method.
     */
    private void drive() {
        // Set actions.
        if (gamepad1.left_bumper)
            state = Teleop.State.INPUT;

        if (gamepad1.right_bumper)
            state = Teleop.State.OUTPUT;

        if (gamepad1.left_trigger > 0.1)
            state = Teleop.State.CARROUSEL_LEFT;

        if (gamepad1.right_trigger > 0.1)
            state = Teleop.State.CARROUSEL_RIGHT;

        // Set weighted drive power to Road Runner.
        robot.drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
    }
}
