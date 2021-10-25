package org.firstinspires.ftc.teamcode.drive.resources;

import java.util.TimerTask;

/**
 * Serves as a timer for the 2021 – 2022 FTC #5421 autonomous algorithm.
 * @see java.util.Timer
 * @see java.util.TimerTask
 */
public class Timer extends java.util.Timer {
    // Data field.
    double time = 0;
    TimerTask timerTask = new TimerTask() {
        @Override
        public void run() {
            time += 0.001;
        }
    };

    /**
     * Constructor.
     */
    public Timer() {
        super("Timer", true);
    }

    /**
     * Start the timer.
     */
    public void start() {
        scheduleAtFixedRate(timerTask, 1, 0);
    }

    /**
     * Stop the timer.
     */
    public void stop() {
        cancel();
    }

    /**
     * Getter method.
     * @return Elapsed time.
     */
    public double getTime() {
        return time;
    }
}
