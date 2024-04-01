// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    public enum ShooterZone {
        Subwoofer,
        Podium,
        Amp,
        Unknown,
        Feeder
    }

    public class ShooterZoneData {
        private final LoggedTunableNumber _lowBound;
        private final LoggedTunableNumber _highBound;
        private LoggedTunableNumber _shooterAngle;
        private LoggedTunableNumber _flywheelSpeed;

        ShooterZoneData(LoggedTunableNumber lowBound, LoggedTunableNumber highBound, LoggedTunableNumber shooterAngle, LoggedTunableNumber flywheelSpeed) {
            this._lowBound = lowBound;
            this._highBound = highBound;
            this._shooterAngle = shooterAngle;
            this._flywheelSpeed = flywheelSpeed;
        }

        // These functions can be called on an enum value to get various bits of data
        public boolean radiusInZone(double radius) {
            return (radius >= _lowBound.get()) && (radius < _highBound.get());
        }

        public double getShooterAngle() {
            return this._shooterAngle.get();
        }

        public double getFlywheelSpeed() {
            return this._flywheelSpeed.get();
        }
    }

    /*
     * ==============================
     * Shooter Zones
     * ==============================
     */
    final ShooterZoneData SubwooferData = new ShooterZoneData(
            ShooterConstants.ZONE_SUBWOOFER_LOW_BOUND,
            ShooterConstants.ZONE_SUBWOOFER_UPPER_BOUND,
            ShooterConstants.ZONE_SUBWOOFER_SHOOTER_ANGLE,
            ShooterConstants.ZONE_SUBWOOFER_FLYWHEEL_SPEED);
    final ShooterZoneData PodiumData = new ShooterZoneData(
            ShooterConstants.ZONE_PODIUM_LOW_BOUND,
            ShooterConstants.ZONE_PODIUM_UPPER_BOUND,
            ShooterConstants.ZONE_PODIUM_SHOOTER_ANGLE,
            ShooterConstants.ZONE_PODIUM_FLYWHEEL_SPEED);
    final ShooterZoneData FeederData = new ShooterZoneData(
            ShooterConstants.ZONE_FEEDER_LOW_BOUND,
            ShooterConstants.ZONE_FEEDER_UPPER_BOUND,
            ShooterConstants.ZONE_FEEDER_SHOOTER_ANGLE,
            ShooterConstants.ZONE_FEEDER_FLYWHEEL_SPEED);
    final ShooterZoneData AmpData = new ShooterZoneData(
            ShooterConstants.ZONE_AMP_LOW_BOUND,
            ShooterConstants.ZONE_AMP_UPPER_BOUND,
            ShooterConstants.ZONE_AMP_SHOOTER_ANGLE,
            ShooterConstants.ZONE_AMP_FLYWHEEL_SPEED);
    final ShooterZoneData UnknownData = new ShooterZoneData(
            ShooterConstants.ZONE_UNKNOWN_LOW_BOUND,
            ShooterConstants.ZONE_UNKNOWN_UPPER_BOUND,
            ShooterConstants.ZONE_UNKNOWN_SHOOTER_ANGLE,
            ShooterConstants.ZONE_UNKNOWN_FLYWHEEL_SPEED);

    private Map<ShooterZone, ShooterZoneData> _zoneDataMappings;

    private static boolean _autoShootEnabled = true;
    private final ShooterIO _shooterIO;
    private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();
    private double _targetFlywheelSpeed = 0;
    private double _targetShooterAngle;
    private ShooterZone _currentShooterZone;
    private ShooterVisualizer _shooterVisualizer = new ShooterVisualizer();
    private final ShooterCommandFactory _shooterCommandFactory = new ShooterCommandFactory(this);
    //private ProfiledPIDController _anglePID;
    private PIDController _anglePID;

    public Shooter(ShooterIO shooterIO) {
        _shooterIO = shooterIO;
        _currentShooterZone = ShooterZone.Unknown;
        // 0.017, 0.00008, 0.25
        
        //_anglePID = new ProfiledPIDController(0.0055, 0.001, 0.0015, new Constraints(40, 70));
        _anglePID = new PIDController(0.0065, 0.0015, 0.0015);
        _anglePID.setIZone(5);

        // Set up the zone mappings
        // This is a funky hack that lets us modify zone data during a match, like angle, 
        // that will stick around until the robot is restarted
        _zoneDataMappings = new HashMap<>();
        _zoneDataMappings.put(ShooterZone.Subwoofer, SubwooferData);
        _zoneDataMappings.put(ShooterZone.Podium, PodiumData);
        _zoneDataMappings.put(ShooterZone.Unknown, UnknownData);
        _zoneDataMappings.put(ShooterZone.Amp, AmpData);
        _zoneDataMappings.put(ShooterZone.Feeder, FeederData);
    }

    public ShooterCommandFactory buildCommand() {
        return this._shooterCommandFactory;
    }

    public void setTargetPosition(ShooterZone zone) {
        setTargetPositionAsAngle(_zoneDataMappings.get(zone).getShooterAngle());
    }

    /**
     * Adjusts the current shooter position by the given amount
     * Amount can be positive or negative; positive will increase the angle, negative will decrease
     * This adjustment will become the new position for whatever zone we are currently in,
     * this change will stay until the code is restarted
     * @param amountInDegrees the amount to change the current angle by
     */
    public void adjustShooterAngle(double amountInDegrees) {
        // _zoneDataMappings.get(_currentShooterZone)._shooterAngle = _zoneDataMappings.get(_currentShooterZone).getShooterAngle() + amountInDegrees;
        // setTargetPositionAsAngle(_zoneDataMappings.get(_currentShooterZone).getShooterAngle());
        // String key = "shooter/zone/" + _currentShooterZone + "/angle";
        // Logger.recordOutput(key,_zoneDataMappings.get(_currentShooterZone).getShooterAngle());
        setTargetPositionAsAngle(_targetShooterAngle + amountInDegrees);
    }

    public void setTargetPositionAsAngle(double angle) {
        if (angle < ShooterConstants.MINIMUM_ANGLE_ENCODER_ANGLE) {
            System.err.println("Invalid angle: Parameter 'angle' must >= MIN_SHOOTER_ANGLE.");
            return;

        } else if (angle > ShooterConstants.MAXIMUM_ANGLE_ENCODER_ANGLE) {
            System.err.println("Invalid angle: Parameter 'angle' must <= MAX_SHOOTER_ANGLE.");
            return;
        } else {
            // if we make a big change in the requested angle, reset the PID controller
            // before moving
            // if(Math.abs(_targetShooterAngle - angle) > 5)
            //     _anglePID.reset(_shooterInputs._angleEncoderPositionDegrees);
            // _anglePID.setGoal(angle);
            _anglePID.setSetpoint(angle);
            // _shooterIO.setTargetPositionAsDegrees(angle);
            _targetShooterAngle = angle;
        }
    }

    /**
     * Get the desired position of the shooter in degrees.
     * This is the same as the setpoint of the PID.
     * 
     * @return The target position of the shooter
     */
    public double getTargetPositionAsAngle() {
        return this._targetShooterAngle;
    }

    /**
     * The current angle of the shooter in degrees. This value is determined
     * directly
     * from the rev encoder on the pivot shaft.
     * 
     * @return shooter angle in degrees
     */
    public double getCurrentPositionInDegrees() {
        // TODO: log an error but do not throw an exception. Not right now at least.
        // Maybe later
        // Garrett 2/26/24

        // if (_shooterInputs._angleEncoderPositionDegrees >=
        // ShooterConstants.MAXIMUM_ANGLE_ENCODER_ANGLE
        // + 10
        // || _shooterInputs._angleEncoderPositionDegrees <=
        // ShooterConstants.MINIMUM_ANGLE_ENCODER_ANGLE
        // - 10) {
        // } else {
        return _shooterInputs._angleEncoderPositionDegrees;
        // }
    }

    /**
     * The current zone the shooter thinks it is in.
     * 
     * @return Current Zone
     */
    public ShooterZone getCurrentZone() {
        return _currentShooterZone;
    }

    // TODO: THIS SHOULD BE SET AS MATH.ABS() ONCE SHOOTER FLYWHEEL PIDS ARE FIXED
    private boolean areFlywheelsAtTargetSpeed() {
        return _targetFlywheelSpeed
                - _shooterInputs._leftFlywheelMotorVelocityRotationsPerMin <= 
                 _targetFlywheelSpeed - _targetFlywheelSpeed * ShooterConstants.FLYWHEEL_SPEED_DEADBAND;
                 // the deadband is a percent. so subtract the deadband perctange times the target from the
                 // target to get the lower bound.
    }

    /**
     * Set the shooter position for a given zone and also set the flywheel speed for
     * that zone
     * 
     * @param zone
     */
    public void runShooterForZone(ShooterZone zone) {
        _currentShooterZone = zone;
        setShooterPositionWithZone(zone);
        setFlywheelSpeed(_zoneDataMappings.get(zone).getFlywheelSpeed());
    }

    public void runShooterForRadius(double radius){
        setFlywheelSpeedWithRadius(radius);
        setShooterPositionWithRadius(radius);
    }

    /**
     * Figure out what zone matches a given radius from the speaker.
     * Returns the unknown zone if no zone matches.
     * 
     * @param radius
     * @return the zone used for that radius
     */
    public ShooterZone getZoneFromRadius(double radius) {
        for (ShooterZone zone : ShooterZone.values()) {
            if (_zoneDataMappings.get(zone).radiusInZone(radius)) {
                return zone;
            }
        }

        return ShooterZone.Unknown;
    }

    public boolean isAutoShootEnabled() {
        return _autoShootEnabled;
    }

    public void setAutoShootEnabled(boolean enabled) {
        _autoShootEnabled = enabled;
    }

    /**
     * Set the position angle of the shooter based on a zone.
     * 
     * @param zone
     */
    public void setShooterPositionWithZone(ShooterZone zone) {
        setTargetPositionAsAngle(_zoneDataMappings.get(zone).getShooterAngle());
    }

    public void setShooterPositionWithRadius(double radius) {
        double angle;
        // only shoot inside this radius
        if (radius <= 4.5) {
            if(radius < 1){
                // the subwoofer is about a meter away from the alliance wall
                // if we think we are less than a meter from the speaker then we
                // are "inside" the subwoofer which is not possible. so harcode ourselves to one
                // meter
                radius = 1;
            }
            angle = -21.02 * Math.log(0.1106 * radius);
            angle -= 2;
            if (radius > 2.0 && radius < 2.75) {
                angle -= 2.25;
            } else if (radius >= 2.75 && radius < 3.75) {
                angle -= 1.25;
            } 
            // else if (radius >= 3.75) {
            //     angle -= 0.;
            // }
            Logger.recordOutput("Shooter/RegressionEstimatedAngle", angle);
            setTargetPositionAsAngle(angle);
        }

    }

    public boolean shooterInPosition() {
        return Math.abs(_targetShooterAngle
                - getCurrentPositionInDegrees()) <= ShooterConstants.ANGLE_ENCODER_DEADBAND_DEGREES;
    }

    /**
     * Set the fly wheel speed to zero.
     */
    public void stopFlywheels() {
        _shooterIO.stopFlywheels();
    }

    public void setFlywheelSpeedWithZone(ShooterZone zone) {
        setFlywheelSpeed(_zoneDataMappings.get(zone).getFlywheelSpeed());
    }

    public void setFlywheelSpeedWithRadius(double radiusInMeters) {
        double speed = 2000;
        if (radiusInMeters <= 4 && radiusInMeters > 2) {
            speed = 1600;
        } else if (radiusInMeters <= 2) {
            speed = 1200;
        }
        setFlywheelSpeed(speed);
    }

    public void setFlywheelSpeed(double speedInRPM) {
        this._targetFlywheelSpeed = speedInRPM;
        _shooterIO.setFlywheelSpeedRPM(speedInRPM);
    }

    public boolean isReady() {
        return shooterInPosition() && areFlywheelsAtTargetSpeed();
    }
    

    public void runAnglePID() {
        double output = calcAnglePID();

        _shooterIO.setAngleMotorSpeed(output);
    }

    private double calcAnglePID() {
        return _anglePID.calculate(_shooterInputs._angleEncoderPositionDegrees) 
            + calcFeedforward();
    }

    private double calcFeedforward() {
       return (ShooterConstants.SHOOTER_FEEDFORWARD_CONSTANT * Math.cos(
                Units.degreesToRadians(_shooterInputs._angleEncoderPositionDegrees 
                + ShooterConstants.ANGLE_FROM_ROBOT_ZERO_TO_GROUND_DEGREES)));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _shooterIO.updateInputs(_shooterInputs);
        Logger.processInputs("Shooter", _shooterInputs);

        // VISUALIZATION
        _shooterVisualizer.update(_shooterInputs._angleEncoderPositionDegrees);

        // LOGGING
        Logger.recordOutput("Shooter/FlywheelSetpoint", this._targetFlywheelSpeed);
        Logger.recordOutput("Mechanism2D/Shooter", _shooterVisualizer.getMechanism());
        Logger.recordOutput("Shooter/isReady", this.isReady());
        Logger.recordOutput("Shooter/inPosition", this.shooterInPosition());
        Logger.recordOutput("Shooter/areFlywheelsAtTargetSpeed", this.areFlywheelsAtTargetSpeed());
        Logger.recordOutput("Shooter/PIDSetpoint", _anglePID.getSetpoint());
        Logger.recordOutput("Shooter/TargetShooterAngle", _targetShooterAngle);
        Logger.recordOutput("Shooter/AnglePIDCalculatedOutput", calcAnglePID());
        Logger.recordOutput("Shooter/isAutoShootEnabled",this.isAutoShootEnabled());
    }
}
