/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                       
 */

/* Open Source Software - may be modified and shared by FRC teams. The code   */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project.                                                       */

/*----------------------------------------------------------------------------*/


package org.usfirst.frc.team6419.robot;



import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import org.usfirst.frc.team6419.robot.commands.auto.SwitchAuto;

import org.usfirst.frc.team6419.robot.commands.core.DriveToPoint;

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

	public static MecanumDrivetrain drivetrain = new MecanumDrivetrain();

	public static Lift lift = new Lift(Config.liftMotorPin);

	public static Intake intake = new Intake(Config.intakeSolenoidPin0, Config.intakeSolenoidPin1, Config.intakeR0, Config.intakeR1, Config.intakeL0, Config.intakeL1);
	public static OI m_oi;

	Command m_autonomousCommand;

	SendableChooser<Command> m_chooser = new SendableChooser<>();


	public static void log(String fmt, Object... args) {
		System.out.print("[");
		System.out.print(Timer.getFPGATimestamp());
		System.out.println("] " + String.format(fmt, args));
	}
	
	public static void log(Sendable src, Object msg) {
		System.out.print("[");
		System.out.print(Timer.getFPGATimestamp());
		System.out.println("] " + src.getName() + ": " + String.valueOf(msg));
	}
	
	public static void log(Object msg) {
		System.out.print("[");
		System.out.print(Timer.getFPGATimestamp());
		System.out.println("] " + String.valueOf(msg));
	}
	
	/**
	 
	* This function is run when the robot is first started up and should be

	 * used for any initialization code.

	 */


	@Override

	public void robotInit() {

		m_oi = new OI();

		// chooser.addObject("My Auto", new MyAutoCommand());

		SmartDashboard.putData("Auto mode", m_chooser);

	}


	/**

	 * This function is called once each time the robot enters Disabled mode.

	 * You can use it to reset any subsystem information you want to clear when

	 * the robot is disabled.

	 */

	@Override

	public void disabledInit() {

		drivetrain.configurePID();
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
		
	//	m_autonomousCommand = m_chooser.getSelected();

		
		m_autonomousCommand = new DriveToPoint(0, 10);
		Robot.log( "Auto command " + m_autonomousCommand);
		/*

		 * String autoSelected = SmartDashboard.getString("Auto Selector",

		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand

		 * = new MyAutoCommand(); break; case "Default Auto": default:

		 * autonomousCommand = new ExampleCommand(); break;}
 		 */


		// schedule the autonomous command (example)

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

		// This makes sure that the autonomous stops running when

		// teleop starts running. If you want the autonomous to

		// continue until interrupted by another command, remove

		// this line or comment it out.
	
	
		if (m_autonomousCommand != null) {

			m_autonomousCommand.cancel();

		}
		imu.reset();

	}


	/**

	 * This function is called periodically during operator control.

	 */

	@Override

	public void teleopPeriodic() {

		Scheduler.getInstance().run();

	}


	private int testStage = 0;
	private double testTime, origEnc;
	private boolean testStageRunning;
	
	@Override
	public void testInit() {
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
