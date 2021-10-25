package org.firstinspires.ftc.teamcode.drive.resources;

import java.util.HashMap;

/**
 * Class that stores operation constants.
 */
public class Constants {
    // Time constants for autonomous.
    public static final HashMap<String, Integer> AUTO_TIMES = new HashMap<String, Integer>() {{
        put("cycles", 0);
    }};

    // Power levels for the slide mechanism towards each goal level.
    static final HashMap<String, Integer> SLIDE_POSITIONS = new HashMap<String, Integer>() {{
        put("low", 0); put("middle", 0); put("high", 0);
    }};

    // Intake powers – "on" or "off."
    static final double[] INTAKE_POWERS = new double[] {0, 0};

    // Intake servo positions – "up" or "down."
    static final double[] INTAKE_POSITIONS = new double[] {0, 0};

    // Carriage positions – "open" and "closed."
    static final double[] CARRIAGE_POSITIONS = new double[] {0, 0, 0};

    // Slides powers – "on" or "off."
    static final double[] SLIDES_POWERS = {0, 0};

    // Carrousel positions – drops the ducky.
    static final int CARROUSEL_POSITIONS = 0;

    // Distance needed to collect shipping elements.
    static final double INTAKE_DISTANCE = 0;

    // Carrousel powers – "on" or "off."
    static final int CARROUSEL_POWERS = 0;

    // Color sensor parameter for shutting down intake.
    static final int targetProximity = 0;
}