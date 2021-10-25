package org.firstinspires.ftc.teamcode.drive.extras;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.drive.opModes.Teleop;
import org.firstinspires.ftc.teamcode.drive.resources.FreightFrenzyRobot;

public class Utilities {
    public static final String VUFORIA_KEY = "ASWrbDH/////AAABmTwPU3Gx1Ep8goD+wc3Rc/osG9caJ2DCR/HUV8Fz6I/5P6Wv74pDnLErwt/w0SSsqc7ElZf9qlBclry1g39wKKPIRnibhJ7RFgZJ02WtJoBNULot/M6gtKZnpbAmviNMWeUO/96gKGGVkFDw5bsv51jFDaHDiLOZYHgD/pM3Q7zgMnUIzTKeHn2LrL2D9/eY9f4ZivEoOUYNpbs0g5sxsWUa2+ubmOeZ41J1U0x6/eyaXFGEZpKigan3JSeW03KkFRrEYflel2pyKInMhSE7HTCuhR1zzZzhmNQBtRF0sFZpeopg4Kad+yXgErC7k3aLyKznvbu2L2WOxNeC6FF67Qgh9FzbOMP/homXd62h6EGi";

    public void computerVision(FreightFrenzyRobot robot, HardwareMap hardwareMap) {
        // Get camera ID number.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        // Initialize vuforia parameters.
        VuforiaLocalizer.Parameters parameters =
                new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = Utilities.VUFORIA_KEY;
        parameters.cameraName = robot.webcam;
        parameters.useExtendedTracking = false;

        // Create vuforia engine.
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }


    /*
     * Drive forward.
    private void drive() {
        // Set actions.
        if (gamepad1.a) state = Teleop.State.INPUT;
        if (gamepad1.x) state = Teleop.State.OUTPUT;
        if (gamepad1.y) state = Teleop.State.CARROUSEL_LEFT;
        if (gamepad1.b) state = Teleop.State.CARROUSEL_RIGHT;

        // Define move constants.
        forward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        twist = -gamepad1.right_stick_x;

        // Define default speeds.
        speeds = new double[] {
                (forward + strafe + twist),
                (forward - strafe - twist),
                (forward - strafe + twist),
                (forward + strafe - twist)
        };

        // Define the maximum speed.
        max = Math.abs(speeds[0]);
        for (double speed: speeds) {
            if (max < Math.abs(speed))
                max = Math.abs(speed);
        }

        // Scale the speeds array accordingly.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++)
                speeds[i] /= max;
        }

        // Move robot accordingly.
        if (gamepad2.dpad_up || gamepad1.dpad_up) {
            robot.motors.get("fl").setPower(-.25);
            robot.motors.get("fr").setPower(-.25);
            robot.motors.get("bl").setPower(-.25);
            robot.motors.get("br").setPower(-.25);
        }
        else if (gamepad2.dpad_right || gamepad1.dpad_right){
            robot.motors.get("fl").setPower(-.25);
            robot.motors.get("fr").setPower(.25);
            robot.motors.get("bl").setPower(.25);
            robot.motors.get("br").setPower(-.25);
        }
        else if (gamepad2.dpad_left || gamepad1.dpad_left){
            robot.motors.get("fl").setPower(.25);
            robot.motors.get("fr").setPower(-.25);
            robot.motors.get("bl").setPower(-.25);
            robot.motors.get("br").setPower(.25);
        }
        else if (gamepad2.dpad_down || gamepad1.dpad_down){
            robot.motors.get("fl").setPower(.25);
            robot.motors.get("fr").setPower(.25);
            robot.motors.get("bl").setPower(.25);
            robot.motors.get("br").setPower(.25);
        }
        else {
            robot.motors.get("fl").setPower(speeds[0]);
            robot.motors.get("fr").setPower(speeds[1]);
            robot.motors.get("bl").setPower(speeds[2]);
            robot.motors.get("br").setPower(speeds[3]);
        }
    }
     */

    // It's been fine they are trying but I'm hearing more programming concepts from Joy.
    // thats why i gave them a function yknow
    // More from Joy, she is writing and leading the conversation.
    // This is enough from my end to confirm who we want
}
