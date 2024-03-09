package frc.robot.util;

public interface Tunable {
    /** When run should update the tunable numbers used in that class through being run through some other method that will run such as updateInputs() */
    public void updateTunables();
}