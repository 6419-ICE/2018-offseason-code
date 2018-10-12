/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6419.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team6419.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	/*
	 * Right stick is omnidirectional movement, left stick x-axis is turning.
	 * 
	 * Buttons are the same for both joysticks.
	 * Button 2: Toggle intake
	 * Button 5: Open intake
	 * Button 3: Close intake
	 * Button 4: Succ (intake cube)
	 * Button 6: Spit (outtake cube)
	 * Hat switch up: Raise lift
	 * Hat switch down: Lower lift
	 */
	
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	
	private Joystick left, right, mechanism;
	private JoystickButton straightDrive, straightDriveAlt,
					intake, intakeAlt, intakeMechanism,
					outtake, outtakeAlt, outtakeMechanism,
					openIntake, openIntakeAlt, openIntakeMechanism,
					closeIntake, closeIntakeAlt, closeIntakeMechanism,
					toggleIntake, toggleIntakeAlt, toggleIntakeMechanism;
	private Command intakeCmd, outtakeCmd;
	
	public OI() {
		left = new Joystick(0);
		right = new Joystick(1);
		mechanism = new Joystick(2);
		straightDrive = new JoystickButton(right, 1);
		straightDriveAlt = new JoystickButton(left, 1);
		straightDrive.whileHeld(new StraightDrive());
		straightDriveAlt.whileHeld(new StraightDrive());
		SmartDashboard.putData("Reset Gyro", new ResetGyro());
		SmartDashboard.putData("Sync PIDs", new SyncPIDTunings());
		SmartDashboard.putData("Calibrate IMU", new CalibrateIMU());
		SmartDashboard.putData("Test Encoders", new TestEncoders());
		
		intake = new JoystickButton(right, 4);
		intakeAlt = new JoystickButton(left, 4);
		// todo find the right buttons
		intakeMechanism = new JoystickButton(mechanism, 4);
		outtake = new JoystickButton(right, 6);
		outtakeAlt = new JoystickButton(left, 6);
		outtakeMechanism = new JoystickButton(mechanism, 6);
		openIntake = new JoystickButton(right, 5);
		openIntakeAlt = new JoystickButton(left, 5);
		openIntakeMechanism = new JoystickButton(mechanism, 5);
		closeIntake = new JoystickButton(right, 3);
		closeIntakeAlt = new JoystickButton(left, 3);
		closeIntakeMechanism = new JoystickButton(mechanism, 3);
		toggleIntake = new JoystickButton(right, 2);
		toggleIntakeAlt = new JoystickButton(left, 2);
		toggleIntakeMechanism = new JoystickButton(mechanism, 2);
		
		openIntake.whenPressed(new OpenIntake());
		openIntakeAlt.whenPressed(new OpenIntake());
		openIntakeMechanism.whenPressed(new OpenIntake());
		closeIntake.whenPressed(new CloseIntake());
		closeIntakeAlt.whenPressed(new CloseIntake());
		closeIntakeMechanism.whenPressed(new CloseIntake());
		toggleIntake.whenPressed(new ToggleIntake());
		toggleIntakeAlt.whenPressed(new ToggleIntake());
		toggleIntakeMechanism.whenPressed(new ToggleIntake());
		intake.whileHeld(new IntakeCubeWithoutUltrasonic());
		intakeAlt.whileHeld(new IntakeCubeWithoutUltrasonic());
		intakeMechanism.whileHeld(new IntakeCubeWithoutUltrasonic());
		outtake.whileHeld(new OuttakeCube());
		outtakeAlt.whileHeld(new OuttakeCube());
		outtakeMechanism.whileHeld(new OuttakeCube());
	}
	
	/**
	 * Get right joystick object
	 * @return right joystick
	 */
	public Joystick getRightJoystick() {
		return right;
	}
	
	/**
	 * Get the value of the right joystick x axis
	 * @return value
	 */
	public double getRightX() {
		return right.getRawAxis(0);
	}
	
	/**
	 * Get the value of the right joystick y axis
	 * @return value
	 */
	public double getRightY() {
		return right.getRawAxis(1);
	}
	
	/**
	 * Get the value of the right joystick z axis
	 * @return value
	 */
	public double getRightZ() {
		if (right.getAxisCount() >= 3) {
			return right.getRawAxis(2);
		}
		System.out.println("Warning: Right joystick does not have 3 axes, but is being queried as if it did");
		return 0;
	}
	
	/**
	 * Get the left joystick object
	 * @return left joystick
	 */
	public Joystick getLeftJoystick() {
		return left;
	}
	
	/**
	 * Get the value of the left joystick x axis
	 * @return value
	 */
	public double getLeftX() {
		return left.getRawAxis(0);
	}
	
	/**
	 * Get the value of the left joystick y axis
	 * @return value
	 */
	public double getLeftY() {
		return left.getRawAxis(1);
	}
	
	/**
	 * Get the value of the left joystick z axis
	 * @return value
	 */
	public double getLeftZ() {
		if (left.getAxisCount() >= 3) {
			return left.getRawAxis(2);
		}
		System.out.println("Warning: Left joystick does not have 3 axes, but is being queried as if it did");
		return 0;
	}
}
