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
 *
 */
public class MecanumDrivetrain extends PIDSubsystem {
	
	private WPI_TalonSRX frontRight, frontLeft, backLeft, backRight;
	private boolean lastTurn, turningPidActive;
	
	public boolean fieldRelative;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public MecanumDrivetrain() {
		super(0, 0, 0, 5, 0);
		frontLeft = new WPI_TalonSRX(RobotMap.FRONT_LEFT);
		frontRight = new WPI_TalonSRX(RobotMap.FRONT_RIGHT);
		backLeft = new WPI_TalonSRX(RobotMap.BACK_LEFT);
		backRight = new WPI_TalonSRX(RobotMap.BACK_RIGHT);
		
		//config the encoders here.
		//WARNING I configured the sensors to use absolute values instead of relative.
		//WARNING Two of the encoders will be backwards and need inverted.  
		frontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		frontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		backLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		backRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		frontRight.setInverted(true);
		backRight.setInverted(true);
		
		frontLeft.setSensorPhase(true);
		frontRight.setSensorPhase(true);
		backLeft.setSensorPhase(true);
		backRight.setSensorPhase(true);
		
		frontLeft.setNeutralMode(NeutralMode.Brake);
		frontRight.setNeutralMode(NeutralMode.Brake);
		backLeft.setNeutralMode(NeutralMode.Brake);
		backRight.setNeutralMode(NeutralMode.Brake);
		
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
		
		frontLeft.selectProfileSlot(0, 0);
		frontRight.selectProfileSlot(0, 0);
		backLeft.selectProfileSlot(0, 0);
		backRight.selectProfileSlot(0, 0);
		
		frontLeft.configAllowableClosedloopError(0, 10, 0);
		frontRight.configAllowableClosedloopError(0, 10, 0);
		backLeft.configAllowableClosedloopError(0, 10, 0);
		backRight.configAllowableClosedloopError(0, 10, 0);
		
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
		
		//frontRight.setSensorPhase(true);
		
		fieldRelative = false;
		/*if (!SmartDashboard.containsKey("Field Relative")) {
			SmartDashboard.putBoolean("Field Relative", fieldRelative);
		}
		fieldRelative = SmartDashboard.getBoolean("Field Relative", false);*/
		configurePID();
		stop();
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new HandleMecanumDrive());
    }
    
    /**
     * Sets the coefficients for the PID loop. I moved this to a method to reinitialize the tunings so the
     * robot would update from the preferences.
     */
    public void configurePID() {
		System.out.println("Synchronizing PID tunings");
		
		if (!Preferences.getInstance().containsKey("turningPid-P")) {
			Preferences.getInstance().putDouble("turningPid-P", 0.1);
		}
		getPIDController().setP(Preferences.getInstance().getDouble("turningPid-P", 0.1));
		
		if (!Preferences.getInstance().containsKey("turningPid-I")) {
			Preferences.getInstance().putDouble("turningPid-I", 0.1);
		}
		getPIDController().setI(Preferences.getInstance().getDouble("turningPid-I", 0.1));
		
		if (!Preferences.getInstance().containsKey("turningPid-D")) {
			Preferences.getInstance().putDouble("turningPid-D", 0.1);
		}
		getPIDController().setD(Preferences.getInstance().getDouble("turningPid-D", 0.1));
		
		if (!Preferences.getInstance().containsKey("turningPid-F")) {
			Preferences.getInstance().putDouble("turningPid-F", 0.1);
		}
		getPIDController().setF(Preferences.getInstance().getDouble("turningPid-F", 0));
		
		getPIDController().setOutputRange(-1, 1);
		getPIDController().setAbsoluteTolerance(0.05 * Math.PI);
    }
    
    /**
     * set the encoders positions to 0. 
     */
    public void resetEncoders() {
    	Robot.log(this, "Encoders reset");
    	frontLeft.setSelectedSensorPosition(0, 0, 10);
    	frontRight.setSelectedSensorPosition(0, 0, 10);
    	backLeft.setSelectedSensorPosition(0, 0, 10);
    	backRight.setSelectedSensorPosition(0, 0, 10);
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
     * @param heading
     */
    public void setTargetHeading(double heading) {
    	Robot.log(this, String.format("Target heading set to %f", heading));
    	turningPidActive = true;
    	getPIDController().reset();
    	getPIDController().enable();
    	getPIDController().setSetpoint(heading);
    }
    
    public boolean targetHeadingReached() {
    	return getPIDController().onTarget();
    }
    
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
    
    public void drive(double x, double y, double rot) {
    	double theta = Math.atan2(y, x) + Math.PI / 2.0;
    	if (fieldRelative) {
    		theta += Robot.imu.getHeading() % (2.0 * Math.PI);
    	}
    	double r = Math.hypot(x, y);
    	double _rot = Utils.applyDeadband(rot, 0.05);
    	if (Utils.withinRange(rot, -0.1, 0.1)) {
    		if (lastTurn) {
    			setTargetHeading(Robot.imu.getHeading());
    		}
    		lastTurn = false;
    	} else {
    		turningPidActive = false;
    		lastTurn = true;
    	}
    	if (turningPidActive) {
    			_rot = getPIDController().get();
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
    	double _rot = getPIDController().get();
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
    	return Math.abs(frontLeft.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold && Math.abs(frontLeft.getClosedLoopTarget(0) - frontLeft.getSelectedSensorPosition(0)) < Config.driveTalonEncoderErrorThreshold;
    }
    
    public boolean frTargetReached() {
    	return Math.abs(frontRight.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold && Math.abs(frontRight.getClosedLoopTarget(0) - frontRight.getSelectedSensorPosition(0)) < Config.driveTalonEncoderErrorThreshold;
    }
    
    public boolean blTargetReached() {
    	return Math.abs(backLeft.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold && Math.abs(backLeft.getClosedLoopTarget(0) - backLeft.getSelectedSensorPosition(0)) < Config.driveTalonEncoderErrorThreshold;
    }
    
    public boolean brTargetReached() {
    	return Math.abs(backRight.getSelectedSensorVelocity(0)) < Config.driveTalonSpeedThreshold;// && Math.abs(backRight.getClosedLoopError(0)) < Config.driveTalonEncoderErrorThreshold;
    }
    
    public double getFLSpeed() {
    	return frontLeft.getSelectedSensorVelocity(0);
    }
    
    public double getFRSpeed() {
    	return frontRight.getSelectedSensorVelocity(0);
    }
    
    public double getBLSpeed() {
    	return backLeft.getSelectedSensorVelocity(0);
    }
    
    public double getBRSpeed() {
    	return backRight.getSelectedSensorVelocity(0);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
    	System.out.println("Initializing sendable");
    	builder.addBooleanProperty("PID active", () -> turningPidActive, null);
    	builder.addDoubleArrayProperty("FL", () -> new double[] {frontLeft.get(), frontLeft.getSelectedSensorPosition(0), frontLeft.getSelectedSensorVelocity(0), (frontLeft.getControlMode().value == 1) ? frontLeft.getClosedLoopTarget(0) : 0}, null);
    	builder.addDoubleArrayProperty("FR", () -> new double[] {frontRight.get(), frontRight.getSelectedSensorPosition(0), frontRight.getSelectedSensorVelocity(0), (frontRight.getControlMode().value == 1) ? frontRight.getClosedLoopTarget(0) : 0}, null);
    	builder.addDoubleArrayProperty("BL", () -> new double[] {backLeft.get(), backLeft.getSelectedSensorPosition(0), backLeft.getSelectedSensorVelocity(0), (backLeft.getControlMode().value == 1) ? backLeft.getClosedLoopTarget(0) : 0}, null);
    	builder.addDoubleArrayProperty("BR", () -> new double[] {backRight.get(), backRight.getSelectedSensorPosition(0), backRight.getSelectedSensorVelocity(0), (backRight.getControlMode().value == 1) ? backRight.getClosedLoopTarget(0) : 0}, null);
    	System.out.println("Properties added");
    	super.initSendable(builder);
    }

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return Robot.imu.getHeading();
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		
	}
}

