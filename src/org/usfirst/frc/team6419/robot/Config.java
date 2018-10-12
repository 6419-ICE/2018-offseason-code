package org.usfirst.frc.team6419.robot;

import org.opencv.core.Mat;

public class Config {
	
	// 
	public static final int drivetrainFrontLeftMotorPin = 2;
	public static final int drivetrainFrontRightMotorPin = 1;
	public static final int drivetrainBackLeftMotorPin = 4;
	public static final int drivetrainBackRightMotorPin = 3;
	
	// Wheel diameter in inches
	public static final double wheelDiameter = 6;
	
	// Wheel circumference in inches
	public static final double wheelCircumference = Math.PI * wheelDiameter;
	
	// Threshold for when the drive Talons are considered stopped
	public static final double driveTalonSpeedThreshold = 0.1;
	
	// Ticks per revolution for the CTRE Mag Encoders
	public static final int ticksPerRev = 4096;
	
	// Inches per tick of travel
	public static final double inchesPerTick = wheelCircumference / ticksPerRev;
	
	// Ticks per inch of robot travel
	public static final double ticksPerInch = ticksPerRev / wheelCircumference;
	
	// Max speed for the motors in MecanumDrivetrain::driveWithVelocity
	public static final double stpMaxSpeed = 0.1 * ticksPerRev * inchesPerTick;
	
	// Same as above, but in ticks per second
	public static final double stpMaxTickSpeed = 0.1 * ticksPerRev;
	
	// PWM pin connected to the lift Victor SP
	public static final int liftMotorPin = 0;
	
	// Threshold for when the drive Talons are considered to have reached their target
	public static final int driveTalonEncoderErrorThreshold = 150;
	
	// The PCM pins connected to the intake solenoid
	public static final int intakeSolenoidPin0 = 0, intakeSolenoidPin1 = 3;
	
	// PWM pins connected to the intake whe	el controllers
	public static final int intakeR0 = 1, intakeR1 = 2, intakeL0 = 3, intakeL1 = 4;
	
	public static final int intakeUltrasonicInput = 0;
	
	public static final double recalibrationThreshold = Math.toRadians(15);
}
