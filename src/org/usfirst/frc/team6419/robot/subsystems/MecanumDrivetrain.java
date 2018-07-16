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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class MecanumDrivetrain extends Subsystem {
	
	private WPI_TalonSRX frontRight, frontLeft, backLeft, backRight;
	private PID turningPid;
	
	public PIDTunings tunings;
	public boolean fieldRelative;
	private boolean lastTurn, turningPidActive;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public MecanumDrivetrain() {
		frontLeft = new WPI_TalonSRX(RobotMap.FRONT_LEFT);
		frontRight = new WPI_TalonSRX(RobotMap.FRONT_RIGHT);
		backLeft = new WPI_TalonSRX(RobotMap.BACK_LEFT);
		backRight = new WPI_TalonSRX(RobotMap.BACK_RIGHT);
		
		//config the encoders here.
		//WARNING I configured the sensors to use absolute values instead of relative.
		//WARNING Two of the encoders will be backwards and need inverted.  
		frontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		frontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		backLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		backRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		
		
		frontRight.setInverted(true);
		backRight.setInverted(true);
		
		fieldRelative = false;
		/*if (!SmartDashboard.containsKey("Field Relative")) {
			SmartDashboard.putBoolean("Field Relative", fieldRelative);
		}
		fieldRelative = SmartDashboard.getBoolean("Field Relative", false);*/
		
		tunings = new PIDTunings();
		if (!Preferences.getInstance().containsKey("turningPid-P")) {
			Preferences.getInstance().putDouble("turningPid-P", 0.1);
		}
		tunings.kP = Preferences.getInstance().getDouble("turningPid-P", 0.1);
		
		if (!Preferences.getInstance().containsKey("turningPid-I")) {
			Preferences.getInstance().putDouble("turningPid-I", 0.1);
		}
		tunings.kI = Preferences.getInstance().getDouble("turningPid-I", 0.1);
		
		if (!Preferences.getInstance().containsKey("turningPid-D")) {
			Preferences.getInstance().putDouble("turningPid-D", 0.1);
		}
		tunings.kD = Preferences.getInstance().getDouble("turningPid-D", 0.1);
		
		if (!Preferences.getInstance().containsKey("turningPid-F")) {
			Preferences.getInstance().putDouble("turningPid-F", 0.1);
		}
		tunings.kF = Preferences.getInstance().getDouble("turningPid-F", 0.1);
		tunings.maxCmd = 1.0;
		tunings.minCmd = -1.0;
		turningPid = new PID(tunings);
		turningPid.setDeadband(0.1 * Math.PI);
		turningPidActive = true;
		stop();
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new HandleMecanumDrive());
    }
    
    /**
     * set the encoders positions to 0. 
     */
    public void resetEncoders() {
    	frontLeft.setSelectedSensorPosition(0, 0, 0);
    	frontRight.setSelectedSensorPosition(0, 0, 0);
    	backLeft.setSelectedSensorPosition(0, 0, 0);
    	backRight.setSelectedSensorPosition(0, 0, 0);
    }
       
    /**
     * returns the raw front left encoder distance. 
     * @return
     */
    public double flEncoder() {
    	return frontLeft.getSelectedSensorPosition(0);
    }
    /**
     * returns the raw front right encoder distance.
     * @return
     */
    public double frEncoder() {
    	return frontRight.getSelectedSensorPosition(0);
    }
    /**
     * returns the raw back left encoder distance.
     * @return
     */
    public double blEncoder() {
    	return backLeft.getSelectedSensorPosition(0);
    }
    /**
     * returns the raw back right encoder distance.
     * @return
     */
    public double brEncoder() {
    	return backRight.getSelectedSensorPosition(0);
    }
    
    public void setTargetHeading(double heading) {
    	turningPidActive = true;
    	turningPid.reset();
    	turningPid.setSetpoint(heading);
    }
    
    public boolean targetHeadingReached() {
    	return Utils.withinRange(Robot.imu.getHeading(), turningPid.getSetpoint() - turningPid.getDeadband(), turningPid.getSetpoint() + turningPid.getDeadband());
    }
    
    public void drive(double x, double y, double rot) {
    	double theta = Math.atan2(y, x) + Math.PI / 2.0;
    	if (fieldRelative) {
    		theta += Robot.imu.getHeading() % (2.0 * Math.PI);
    	}
    	double r = Math.hypot(x, y);
    	double _rot = Utils.applyDeadband(rot, 0.2 * Math.PI);
    	if (Utils.withinRange(rot, -0.1 * Math.PI, 0.1 * Math.PI)) {
    		if (lastTurn) {
    			turningPidActive = true;
    			turningPid.reset();
    			turningPid.setSetpoint(Robot.imu.getHeading());
    		}
    		lastTurn = false;
    	} else {
    		turningPidActive = false;
    		lastTurn = true;
    	}
    	if (turningPidActive) {
    		_rot = turningPid.update(Robot.imu.getHeading());
    	}
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
    	frontLeft.set(ControlMode.PercentOutput, fl);
    	frontRight.set(ControlMode.PercentOutput, fr);
    	backLeft.set(ControlMode.PercentOutput, bl);
    	backRight.set(ControlMode.PercentOutput, br);
    }
    
    public double[] driveWithVelocity(double x, double y) {
    	double theta = Math.atan2(y, x) + Math.PI / 2.0;
    	if (fieldRelative) {
    		theta += Robot.imu.getHeading() % (2.0 * Math.PI);
    	}
    	double r = Math.min(Math.hypot(x, y), 1.0);
    	double _rot = turningPid.update(Robot.imu.getHeading());
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
    	
    	frontLeft.set(ControlMode.Velocity, fl * Config.stpMaxTickSpeed);
    	frontRight.set(ControlMode.Velocity, fr * Config.stpMaxTickSpeed);
    	backLeft.set(ControlMode.Velocity, bl * Config.stpMaxTickSpeed);
    	backRight.set(ControlMode.Velocity, br * Config.stpMaxTickSpeed);
    	
    	return new double[] {fl, fr, bl, br};
    }
    /**
     * Sets the power of the motors to 0.
     * TODO: Should we set the motors to brake or leave them on coast?
     */
    public void stop() {
    	frontLeft.set(ControlMode.PercentOutput, 0);
    	frontRight.set(ControlMode.PercentOutput, 0);
    	backLeft.set(ControlMode.PercentOutput, 0);
    	backRight.set(ControlMode.PercentOutput, 0);
    }
    
    public void setFLTarget(double target) {
    	frontLeft.set(ControlMode.Position, target);
    }
    
    public void setFRTarget(double target) {
    	frontRight.set(ControlMode.Position, target);
    }
    
    public void setBLTarget(double target) {
    	backLeft.set(ControlMode.Position, target);
    }
    
    public void setBRTarget(double target) {
    	backRight.set(ControlMode.Position, target);
    }
    
    public boolean flTargetReached() {
    	return Math.abs(frontLeft.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold;
    }
    
    public boolean frTargetReached() {
    	return Math.abs(frontRight.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold;
    }
    
    public boolean blTargetReached() {
    	return Math.abs(backLeft.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold;
    }
    
    public boolean brTargetReached() {
    	return Math.abs(backRight.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold;
    }
}

