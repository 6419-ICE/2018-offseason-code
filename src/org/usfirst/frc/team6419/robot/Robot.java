/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                       
 */

/* Open Source Software - may be modified and shared by FRC teams. The code   */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project.                                                       */

/*----------------------------------------------------------------------------*/


package org.usfirst.frc.team6419.robot;



import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

import org.usfirst.frc.team6419.robot.commands.auto.SwitchAuto;
import org.usfirst.frc.team6419.robot.commands.auto.SwitchAuto2ElectricBogaloo;
import org.usfirst.frc.team6419.robot.commands.auto.SwitchAuto2WithDrive2Point;
import org.usfirst.frc.team6419.robot.commands.auto.SwitchAutoPosition3;
import org.usfirst.frc.team6419.robot.commands.core.DriveToPoint;
import org.usfirst.frc.team6419.robot.commands.core.TimedDriveStraight;
import org.usfirst.frc.team6419.robot.commands.core.TurnToHeading;
import org.usfirst.frc.team6419.robot.subsystems.IMU;
import org.usfirst.frc.team6419.robot.subsystems.Intake;
import org.usfirst.frc.team6419.robot.subsystems.Lift;
import org.usfirst.frc.team6419.robot.subsystems.MecanumDrivetrain;

/**
 
* The VM is configured to automatically run this class, and to call the
 
* functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 If you change the name of this class or the package after
 
* creating this project, you must also update the build.properties file in the
 
* project.
 
*/

public class Robot extends TimedRobot {

	public static IMU imu = new IMU();

	public static MecanumDrivetrain drivetrain = new MecanumDrivetrain(Config.drivetrainFrontLeftMotorPin, Config.drivetrainFrontRightMotorPin, Config.drivetrainBackLeftMotorPin, Config.drivetrainBackRightMotorPin);

	public static Lift lift = new Lift(Config.liftMotorPin);

	public static Intake intake = new Intake(Config.intakeSolenoidPin0, Config.intakeSolenoidPin1, Config.intakeR0, Config.intakeR1, Config.intakeL0, Config.intakeL1, Config.intakeUltrasonicInput);
	
	public static OI m_oi;
	
	private java.util.Timer pidSyncLoop;
	private java.util.Timer imuCalibrationLoop;
	
	private AtomicBoolean hasBeenEnabled = new AtomicBoolean(false);
	private double heading = 0;

	private class PidSyncTask extends TimerTask {
		
		@Override
		public void run() {
			if (DriverStation.getInstance().isDisabled()) {
				Robot.drivetrain.configurePID();
			}
		}
	}
	
	private class IMUCalibrationWatchdog extends TimerTask {
		
		@Override
		public void run() {
			if (DriverStation.getInstance().isDisabled() && !hasBeenEnabled.get()) {
				double drift = Math.abs(imu.getHeading() - heading);
				if (drift > Config.recalibrationThreshold) {
					log("IMU has drifted by %f, recalibrating...", drift);
					imu.calibrate();
					imu.reset();
				}
				heading = imu.getHeading();
			}
			hasBeenEnabled.set(false);
		}
	}
	
	public enum Auto {
		LEFT('L'),
		CENTER('C'),
		RIGHT('R');
		
		private char value;
		
		private Auto(char v) {
			value = v;
		}
		
		public char toChar() {
			return value;
		}
	}

	private Command m_autonomousCommand;
	private SendableChooser<Auto> m_chooser = new SendableChooser<>();
	
	private int testStage = 0;
	private double testTime, origEnc;
	private boolean testStageRunning;

	/**
	 * Print a formatted log entry with a timestamp.
	 * @param fmt format string
	 * @param args format values
	 */
	public static void log(String fmt, Object... args) {
		System.out.print("[");
		System.out.print(Timer.getFPGATimestamp());
		System.out.println("] " + String.format(fmt, args));
	}
	
	/**
	 * Print a log entry with a timestamp and a source tag.
	 * @param src source from which this entry is being logged
	 * @param msg message to log
	 */
	public static void log(Sendable src, Object msg) {
		System.out.print("[");
		System.out.print(Timer.getFPGATimestamp());
		System.out.println("] " + src.getName() + ": " + String.valueOf(msg));
	}
	
	/**
	 * Print a log entry with a timestamp.
	 * @param msg message to log
	 */
	public static void log(Object msg) {
		System.out.print("[");
		System.out.print(Timer.getFPGATimestamp());
		System.out.println("] " + String.valueOf(msg));
	}
	
	/**
	* This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	private PidSyncTask syncTask;
	private IMUCalibrationWatchdog imuCalibrator;

	@Override
	public void robotInit() {
		m_oi = new OI();
		
		m_chooser.addDefault("Center", Auto.CENTER);
		m_chooser.addObject("Left", Auto.LEFT);
		m_chooser.addObject("Right", Auto.RIGHT);
		SmartDashboard.putData("Auto mode", m_chooser);
		if (!Preferences.getInstance().containsKey("HMD-speed")) {
			Preferences.getInstance().putDouble("HMD-speed", 1);
		}
		
		syncTask = new PidSyncTask();
		imuCalibrator = new IMUCalibrationWatchdog();
		
		// Setup PidSyncTask::run to be executed every 2 seconds.
		pidSyncLoop = new java.util.Timer();
		pidSyncLoop.scheduleAtFixedRate(syncTask, 2000, 2000);
		
		imuCalibrationLoop = new java.util.Timer();
		imuCalibrationLoop.scheduleAtFixedRate(imuCalibrator, 500, 500);
		
		//drivetrain.setSpeedLimit(0.5);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */

	@Override
	public void disabledInit() {
		drivetrain.disable();
		testStage = 0;
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */

	@Override
	public void autonomousInit() {
		hasBeenEnabled.set(true);
		imu.reset();
		Auto selected = m_chooser.getSelected();
		if (selected == Auto.LEFT || selected == Auto.RIGHT) {
			m_autonomousCommand = new SwitchAutoPosition3(selected);
		//} else if(selected == "C"){
		//	m_autonomousCommand = new SwitchAuto2ElectricBogaloo();
		} else if (selected == Auto.CENTER) {
			m_autonomousCommand = new SwitchAuto2WithDrive2Point();
		}
		Robot.log("Selected auto: " + m_autonomousCommand.getName());
		
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}	

	@Override
	public void teleopInit() {
		hasBeenEnabled.set(true);
		
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		
		imu.reset();
		drivetrain.setTargetHeading(0);
	}

	/**
	 * This function is called periodically during operator control.
	 */

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
	
	@Override
	public void testInit() {
		hasBeenEnabled.set(true);
		testTime = Timer.getFPGATimestamp();
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	
	@Override
	public void testPeriodic() {
		/*if (testStage == 0) {
			if (testStageRunning) {
				if (Timer.getFPGATimestamp() >= testTime) {
					if (origEnc < drivetrain.flEncoder()) {
						Robot.log("Test: FL: OK");
					} else {
						Robot.log("Test: FL: FAIL");
					}
					drivetrain.setFLPower(0);
					testStage++;
					testStageRunning = false;
					testTime = 0;
				}
			} else {
				log("Test: Stage " + testStage);
				origEnc = drivetrain.flEncoder();
				testTime = Timer.getFPGATimestamp() + 2;
				testStageRunning = true;
				drivetrain.setFLPower(1);
			}
		} else if (testStage == 1) {
			if (testStageRunning) {
				if (Timer.getFPGATimestamp() >= testTime) {
					if (origEnc < drivetrain.frEncoder()) {
						Robot.log("Test: FR: OK");
					} else {
						Robot.log("Test: FR: FAIL");
					}
					drivetrain.setFRPower(0);
					testStage++;
					testStageRunning = false;
					testTime = 0;
				}
			} else {
				log("Test: Stage " + testStage);
				origEnc = drivetrain.frEncoder();
				testTime = Timer.getFPGATimestamp() + 2;
				testStageRunning = true;
				drivetrain.setFRPower(1);
			}
		} else if (testStage == 2) {
			if (testStageRunning) {
				if (Timer.getFPGATimestamp() >= testTime) {
					if (origEnc < drivetrain.blEncoder()) {
						Robot.log("Test: BL: OK");
					} else {
						Robot.log("Test: BL: FAIL");
					}
					drivetrain.setBLPower(0);
					testStage++;
					testStageRunning = false;
					testTime = 0;
				}
			} else {
				log("Test: Stage " + testStage);
				origEnc = drivetrain.blEncoder();
				testTime = Timer.getFPGATimestamp() + 2;
				testStageRunning = true;
				drivetrain.setBLPower(1);
			}
		} else if (testStage == 3) {
			if (testStageRunning) {
				if (Timer.getFPGATimestamp() >= testTime) {
					if (origEnc < drivetrain.brEncoder()) {
						Robot.log("Test: BR: OK");
					} else {
						Robot.log("Test: BR: FAIL");
					}
					drivetrain.setBRPower(0);
					testStage++;
					testStageRunning = false;
					testTime = 0;
				}
			} else {
				log("Test: Stage " + testStage);
				origEnc = drivetrain.brEncoder();
				testTime = Timer.getFPGATimestamp() + 2;
				testStageRunning = true;
				drivetrain.setBRPower(1);
			}
		} else if (testStage == 4) {
			if (!testStageRunning) {
				log("Test: Stage " + testStage);
				testStageRunning = true;
				log("Test: Complete");
			}
		}*/
		if (Timer.getFPGATimestamp() >= testTime + 8) {
			drivetrain.setBRPower(0);
		} else if (Timer.getFPGATimestamp() >= testTime + 6) {
			drivetrain.setBLPower(0);
			drivetrain.setBRPower(1);
		} else if (Timer.getFPGATimestamp() >= testTime + 4) {
			drivetrain.setFRPower(0);
			drivetrain.setBLPower(1);
		} else if (Timer.getFPGATimestamp() >= testTime + 2) {
			drivetrain.setFLPower(0);
			drivetrain.setFRPower(1);
		} else {
			drivetrain.setFLPower(1);
		}
	}
}
