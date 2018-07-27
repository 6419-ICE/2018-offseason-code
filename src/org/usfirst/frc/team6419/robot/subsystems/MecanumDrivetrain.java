package org.usfirst.frc.team6419.robot.subsystems;

import java.util.Arrays;

import org.usfirst.frc.team6419.robot.Config;
import org.usfirst.frc.team6419.robot.Robot;
import org.usfirst.frc.team6419.robot.RobotMap;
import org.usfirst.frc.team6419.robot.commands.HandleMecanumDrive;
import org.usfirst.frc.team6419.robot.core.PID;
import org.usfirst.frc.team6419.robot.core.PIDTunings;
import org.usfirst.frc.team6419.robot.core.Utils;

import com.analog.adis16448.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Mecanum drivetrain subsystem.
 */
public class MecanumDrivetrain extends PIDSubsystem {
	
	private WPI_TalonSRX frontRight, frontLeft, backLeft, backRight;
	private boolean lastTurn, turningPidActive;
	private boolean follow = false;
	public boolean fieldRelative;
	private double pidOutput = 0;
	
	public MecanumDrivetrain() {
		// Args: kP, kI, kD, period, kf
		super(0, 0, 0, 0.05, 0);
		
		// Instantiate the Talons
		frontLeft = new WPI_TalonSRX(RobotMap.FRONT_LEFT);
		frontRight = new WPI_TalonSRX(RobotMap.FRONT_RIGHT);
		backLeft = new WPI_TalonSRX(RobotMap.BACK_LEFT);
		backRight = new WPI_TalonSRX(RobotMap.BACK_RIGHT);
		
		// Configure encoders as feedback sensors.  
		frontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		frontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		backLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		backRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		// Invert the right side
		frontRight.setInverted(true);
		backRight.setInverted(true);
	
		// Set sensor phases to make encoder forward be motor forward
		frontLeft.setSensorPhase(false);
		frontRight.setSensorPhase(true);
		backLeft.setSensorPhase(false);
		backRight.setSensorPhase(true);
		
		// Enable brake mode
		frontLeft.setNeutralMode(NeutralMode.Brake);
		frontRight.setNeutralMode(NeutralMode.Brake);
		backLeft.setNeutralMode(NeutralMode.Brake);
		backRight.setNeutralMode(NeutralMode.Brake);
		
		// Configure nominal (neutral) and peak (max) outputs for the motors
		frontLeft.configNominalOutputForward(0, 0);
		frontLeft.configNominalOutputReverse(0, 0);
		frontLeft.configPeakOutputForward(1, 0);
		frontLeft.configPeakOutputReverse(-1, 0);
		
		frontRight.configNominalOutputForward(0, 0);
		frontRight.configNominalOutputReverse(0, 0);
		frontRight.configPeakOutputForward(1, 0);
		frontRight.configPeakOutputReverse(-1, 0);
		
		backLeft.configNominalOutputForward(0, 0);
		backLeft.configNominalOutputReverse(0, 0);
		backLeft.configPeakOutputForward(1, 0);
		backLeft.configPeakOutputReverse(-1, 0);
		
		backRight.configNominalOutputForward(0, 0);
		backRight.configNominalOutputReverse(0, 0);
		backRight.configPeakOutputForward(1, 0);
		backRight.configPeakOutputReverse(-1, 0);
		
		// Select profile slot. Has something to do with PIDs
		frontLeft.selectProfileSlot(0, 0);
		frontRight.selectProfileSlot(0, 0);
		backLeft.selectProfileSlot(0, 0);
		backRight.selectProfileSlot(0, 0);
		
		// Configure Talon PID deadband
		frontLeft.configAllowableClosedloopError(0, 10, 0);
		frontRight.configAllowableClosedloopError(0, 10, 0);
		backLeft.configAllowableClosedloopError(0, 10, 0);
		backRight.configAllowableClosedloopError(0, 10, 0);
		
		// Configure Talon PID tunings
		frontLeft.config_kF(0, 0, 0);
		frontLeft.config_kP(0, 0.1, 0);
		frontLeft.config_kI(0, 0, 0);
		frontLeft.config_kD(0, 0, 0);
		
		frontRight.config_kF(0, 0, 0);
		frontRight.config_kP(0, 0.1, 0);
		frontRight.config_kI(0, 0, 0);
		frontRight.config_kD(0, 0, 0);
		
		backLeft.config_kF(0, 0, 0);
		backLeft.config_kP(0, 0.1, 0);
		backLeft.config_kI(0, 0, 0);
		backLeft.config_kD(0, 0, 0);
		
		backRight.config_kF(0, 0, 0);
		backRight.config_kP(0, 0.1, 0);
		backRight.config_kI(0, 0, 0);
		backRight.config_kD(0, 0, 0);
		
		// Set encoders to start at the absolute position
		int absPos = frontLeft.getSensorCollection().getPulseWidthPosition();
		absPos &= 0xfff;
		absPos *= -1;
		frontLeft.setSelectedSensorPosition(absPos, 0, 0);
		absPos = frontRight.getSensorCollection().getPulseWidthPosition();
		absPos &= 0xfff;
		frontRight.setSelectedSensorPosition(absPos, 0, 0);
		absPos = backLeft.getSensorCollection().getPulseWidthPosition();
		absPos &= 0xfff;
		absPos *= -1;
		backLeft.setSelectedSensorPosition(absPos, 0, 0);
		absPos = backRight.getSensorCollection().getPulseWidthPosition();
		absPos &= 0xfff;
		backRight.setSelectedSensorPosition(absPos, 0, 0);
		
		fieldRelative = false;
		
		// Sync PIDs just in case
		configurePID();
		// Stop the robot
		stop();
	}
	
    public void initDefaultCommand() {
    	setDefaultCommand(new HandleMecanumDrive());
    }
    
    /**
     * Sync PID tunings to the values in Preferences
     */
    public void configurePID() {
		Robot.log(this, "Synchronizing PID tunings");
		if (!Preferences.getInstance().containsKey("turningPid-P")) {
			Preferences.getInstance().putDouble("turningPid-P", 1);
		}
		getPIDController().setP(Preferences.getInstance().getDouble("turningPid-P", 1));
		
		if (!Preferences.getInstance().containsKey("turningPid-I")) {
			Preferences.getInstance().putDouble("turningPid-I", 0);
		}
		getPIDController().setI(Preferences.getInstance().getDouble("turningPid-I", 0));
		
		if (!Preferences.getInstance().containsKey("turningPid-D")) {
			Preferences.getInstance().putDouble("turningPid-D", 1);
		}
		getPIDController().setD(Preferences.getInstance().getDouble("turningPid-D", 1));
		
		if (!Preferences.getInstance().containsKey("turningPid-F")) {
			Preferences.getInstance().putDouble("turningPid-F", 0);
		}
		getPIDController().setF(Preferences.getInstance().getDouble("turningPid-F", 0));
		
		getPIDController().setOutputRange(-1, 1);
		getPIDController().setAbsoluteTolerance(0.025 * Math.PI);
    }
    
    /**
     * Set current encoder positions to 0. 
     */
    public void resetEncoders() {
    	Robot.log(this, "Encoders reset");
    	frontLeft.setSelectedSensorPosition(0, 0, 10);
    	frontRight.setSelectedSensorPosition(0, 0, 10);
    	backLeft.setSelectedSensorPosition(0, 0, 10);
    	backRight.setSelectedSensorPosition(0, 0, 10);
    }
       
    /**
     * Returns the raw front left encoder position. 
     * @return encoder position
     */
    public double flEncoder() {
    	return frontLeft.getSelectedSensorPosition(0);
    }
    
    /**
     * Returns the raw front right encoder position.
     * @return encoder position
     */
    public double frEncoder() {
    	return frontRight.getSelectedSensorPosition(0);
    }
    
    /**
     * Returns the raw back left encoder position.
     * @return encoder position
     */
    public double blEncoder() {
    	return backLeft.getSelectedSensorPosition(0);
    }
    
    /**
     * Returns the raw back right encoder position.
     * @return encoder position
     */
    public double brEncoder() {
    	return backRight.getSelectedSensorPosition(0);
    }
    
    /**
     * Set a soft[ware] speed limit, in the range [-1, 1]
     * @param limit max absolute output
     */
    public void setSpeedLimit(double limit) {
    	frontLeft.configPeakOutputForward(limit, 10);
    	frontLeft.configPeakOutputReverse(-limit, 10);
    	frontRight.configPeakOutputForward(limit, 10);
    	frontRight.configPeakOutputReverse(-limit, 10);
    	backLeft.configPeakOutputForward(limit, 10);
    	backLeft.configPeakOutputReverse(-limit, 10);
    	backRight.configPeakOutputForward(limit, 10);
    	backRight.configPeakOutputReverse(-limit, 10);
    }
    
    /**
     * Sets a target heading for the robot to follow.
     * @param heading target heading, in radians
     */
    public void setTargetHeading(double heading) {
    	Robot.log(this, String.format("Target heading set to %f", heading));
    	turningPidActive = true;
    	getPIDController().reset();
    	getPIDController().setSetpoint(heading);
    	getPIDController().enable();
    }
    
    /**
     * Returns true when the turning PID has stabilized at its target
     * @return true when at target, false otherwise
     */
    public boolean targetHeadingReached() {
    	return getPIDController().onTarget();
    }
    
    /**
     * Manually enable or disable the turning PID
     * @param enabled true -> enabled, false -> disabled
     */
    public void setTurningPidEnabled(boolean enabled) {
    	turningPidActive = enabled;
    	if (enabled) {
    		Robot.log(this, "Turning PID enabled");
    		getPIDController().enable();
    	} else {
    		Robot.log(this, "Turning PID disabled");
    		getPIDController().disable();
    	}
    }
    
    /**
     * Drive in the direction <x, y> while turning according to rot.
     * @param x x component of desired motion vector. Range: [-1, 1]
     * @param y y component of desired motion vector. Range: [-1, 1]
     * @param rot rotation speed. Range: [-1, 1]
     */
    public void drive(double x, double y, double rot) {
    	// Calculate the polar representation of the point (x, y)
    	double theta = Math.atan2(y, x) + Math.PI / 2.0;
    	if (fieldRelative) {
    		theta += Robot.imu.getHeading() % (2.0 * Math.PI);
    	}
    	double r = Math.hypot(x, y);
    	
    	// Apply a small deadband to rot
    	double _rot = Utils.applyDeadband(rot, 0.05);
    	
    	// Disable the turning PID while turning manually, but re-enable it
    	// when we stop turning, setting the new target to the current heading
    	// at the time of stopping
    	if (Utils.withinRange(rot, -0.1, 0.1)) {
    		if (lastTurn) {
    			setTargetHeading(Robot.imu.getHeading());
    		}
    		lastTurn = false;
    	} else {
    		turningPidActive = false;
    		lastTurn = true;
    	}
    	
    	// If the turning PID is active/enabled, use it's output
    	if (turningPidActive) {
    		_rot = pidOutput;
    	}
    	
    	// Calculate the motor speeds. They range from about -2.5 to about 2.5
    	double fl = r * Math.sin(theta + Math.PI / 4.0) + _rot;
    	double fr = r * Math.cos(theta + Math.PI / 4.0) - _rot;
    	double bl = r * Math.cos(theta + Math.PI / 4.0) + _rot;
    	double br = r * Math.sin(theta + Math.PI / 4.0) - _rot;
    	
    	// If any of the absolute motor speeds are greater than 1, normalize the speeds
    	// by dividing all of them by the absolute max speed.
    	double[] vectors = {Math.abs(fl), Math.abs(fr), Math.abs(bl), Math.abs(br)};
    	Arrays.sort(vectors);
    	if (vectors[0] > 1.0) {
    		fl /= vectors[0];
    		fr /= vectors[0];
    		bl /= vectors[0];
    		br /= vectors[0];
    	}
    	
    	// Set the motor speeds to the calculated speeds
    	frontLeft.set(ControlMode.PercentOutput, fl);
    	frontRight.set(ControlMode.PercentOutput, fr);
    	backLeft.set(ControlMode.PercentOutput, bl);
    	backRight.set(ControlMode.PercentOutput, br);
    }
    
    /**
     * Drive with constant wheel velocities
     * @param x x component of desired motion vector
     * @param y y component of desired motion vector
     * @return wheel velocities
     */
    public double[] driveWithVelocity(double x, double y) {
    	// Calculate the polar representation of (x, y)
    	double theta = Math.atan2(y, x) + Math.PI / 2.0;
    	if (fieldRelative) {
    		theta += Robot.imu.getHeading() % (2.0 * Math.PI);
    	}
    	double r = Math.min(Math.hypot(x, y), 1.0);
    	
    	// Get turning PID output;
    	double _rot = pidOutput;
    	
    	// Calculate and normalize motor speeds
    	double fl = r * Math.sin(theta + Math.PI / 4.0) + _rot;
    	double fr = r * Math.cos(theta + Math.PI / 4.0) - _rot;
    	double bl = r * Math.cos(theta + Math.PI / 4.0) + _rot;
    	double br = r * Math.sin(theta + Math.PI / 4.0) - _rot;
    	double[] vectors = {Math.abs(fl), Math.abs(fr), Math.abs(bl), Math.abs(br)};
    	Arrays.sort(vectors);
    	if (vectors[0] > 1.0) {
    		fl /= vectors[0];
    		fr /= vectors[0];
    		bl /= vectors[0];
    		br /= vectors[0];
    	}
    	
    	// Set closed-loop target velocity to motor speeds * max motor speed
    	frontLeft.set(ControlMode.Velocity, fl * Config.stpMaxTickSpeed);
    	frontRight.set(ControlMode.Velocity, fr * Config.stpMaxTickSpeed);
    	backLeft.set(ControlMode.Velocity, bl * Config.stpMaxTickSpeed);
    	backRight.set(ControlMode.Velocity, br * Config.stpMaxTickSpeed);
    	
    	// Return motor speeds
    	return new double[] {fl, fr, bl, br};
    }
    /**
     * Sets the power of the motors to 0.
     */
    public void stop() {
    	Robot.log(this, "Stopping drive");
    	frontLeft.set(ControlMode.PercentOutput, 0);
    	frontRight.set(ControlMode.PercentOutput, 0);
    	backLeft.set(ControlMode.PercentOutput, 0);
    	backRight.set(ControlMode.PercentOutput, 0);
    }
    
    /**
     * Set front left target
     * @param target
     */
    public void setFLTarget(double target) {
    	Robot.log(this, "Front left target: " + target);
    	frontLeft.set(ControlMode.Position, -target);
    }
    
    /**
     * Set front right target
     * @param target
     */
    public void setFRTarget(double target) {
    	Robot.log(this, "Front right Target: " + target);
    	frontRight.set(ControlMode.Position, target);
    }
    
    /**
     * Set back left target
     * @param target
     */
    public void setBLTarget(double target) {
    	Robot.log(this, "Back Left Target: " + target);
    	backLeft.set(ControlMode.Position, -target);
    }
    
    /**
     * Set back right target
     * @param target
     */
    public void setBRTarget(double target) {
    	Robot.log(this, "Back Right target: " + target);
    	backRight.set(ControlMode.Position, target);
    }
    
    /**
     * Has the front left wheel reached it's target?
     * @return true if target reached
     */
    public boolean flTargetReached() {
    	return Math.abs(frontLeft.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold && Math.abs(frontLeft.getClosedLoopTarget(0) - frontLeft.getSelectedSensorPosition(0)) < Config.driveTalonEncoderErrorThreshold;
    }
    
    /**
     * Has the front right wheel reached it's target?
     * @return true if target reached
     */
    public boolean frTargetReached() {
    	return Math.abs(frontRight.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold && Math.abs(frontRight.getClosedLoopTarget(0) - frontRight.getSelectedSensorPosition(0)) < Config.driveTalonEncoderErrorThreshold;
    }
    
    /**
     * Has the back left wheel reached it's target?
     * @return true if target reached
     */
    public boolean blTargetReached() {
    	return Math.abs(backLeft.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold && Math.abs(backLeft.getClosedLoopTarget(0) - backLeft.getSelectedSensorPosition(0)) < Config.driveTalonEncoderErrorThreshold;
    }
    
    /**
     * Has the back right wheel reached it's target?
     * @return true if target reached
     */
    public boolean brTargetReached() {
    	return Math.abs(backRight.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold;// && Math.abs(backRight.getClosedLoopError(0)) < Config.driveTalonEncoderErrorThreshold;
    }
    
    /**
     * Get front left wheel speed
     * @return speed
     */
    public double getFLSpeed() {
    	return frontLeft.getSelectedSensorVelocity(0);
    }
    
    /**
     * Get front right wheel speed
     * @return speed
     */
    public double getFRSpeed() {
    	return frontRight.getSelectedSensorVelocity(0);
    }
    
    /**
     * Get back left wheel speed
     * @return speed
     */
    public double getBLSpeed() {
    	return backLeft.getSelectedSensorVelocity(0);
    }
    
    /**
     * Get back right wheel speed
     * @return speed
     */
    public double getBRSpeed() {
    	return backRight.getSelectedSensorVelocity(0);
    }
    
    /**
     * Set front left power
     * @param power
     */
    public void setFLPower(double power) {
    	frontLeft.set(ControlMode.PercentOutput, power);
    }
    
    /**
     * Set front right power
     * @param power
     */
    public void setFRPower(double power) {
    	frontRight.set(ControlMode.PercentOutput, power);
    }
    
    /**
     * Set back left power
     * @param power
     */
    public void setBLPower(double power) {
    	backLeft.set(ControlMode.PercentOutput, power);
    }
    
    /**
     * Set back right power
     * @param power
     */
    public void setBRPower(double power) {
    	backRight.set(ControlMode.PercentOutput, power);
    }
    
    /**
     * Initialize MecanumDrivetrain as a Sendable
     */
    @Override
    public void initSendable(SendableBuilder builder) {
    	Robot.log(this, "Initializing sendable");
    	builder.addBooleanProperty("PID active", () -> turningPidActive, null);
    	builder.addDoubleArrayProperty("FL", () -> new double[] {frontLeft.get(), frontLeft.getSelectedSensorPosition(0), frontLeft.getSelectedSensorVelocity(0), (frontLeft.getControlMode().value == 1) ? frontLeft.getClosedLoopTarget(0) : 0}, null);
    	builder.addDoubleArrayProperty("FR", () -> new double[] {frontRight.get(), frontRight.getSelectedSensorPosition(0), frontRight.getSelectedSensorVelocity(0), (frontRight.getControlMode().value == 1) ? frontRight.getClosedLoopTarget(0) : 0}, null);
    	builder.addDoubleArrayProperty("BL", () -> new double[] {backLeft.get(), backLeft.getSelectedSensorPosition(0), backLeft.getSelectedSensorVelocity(0), (backLeft.getControlMode().value == 1) ? backLeft.getClosedLoopTarget(0) : 0}, null);
    	builder.addDoubleArrayProperty("BR", () -> new double[] {backRight.get(), backRight.getSelectedSensorPosition(0), backRight.getSelectedSensorVelocity(0), (backRight.getControlMode().value == 1) ? backRight.getClosedLoopTarget(0) : 0}, null);
    	Robot.log(this, "Properties added");
    	super.initSendable(builder);
    }

	@Override
	protected double returnPIDInput() {
		return Robot.imu.getHeading();
	}

	@Override
	protected void usePIDOutput(double output) {
		Robot.log(this, "PID output: " + output);
		pidOutput = output;
	}
}

