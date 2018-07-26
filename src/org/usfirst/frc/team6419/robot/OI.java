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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team6419.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
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
	
	Joystick left, right;
	JoystickButton straightDrive, intake, outtake, openIntake, closeIntake;
	
	public OI() {
		left = new Joystick(0);
		right = new Joystick(1);
		straightDrive = new JoystickButton(right, 1);
		straightDrive.whileHeld(new StraightDrive());
		SmartDashboard.putData("Reset Gyro", new ResetGyro());
		SmartDashboard.putData("Sync PIDs", new SyncPIDTunings());
		SmartDashboard.putData("Calibrate IMU", new CalibrateIMU());
		SmartDashboard.putData("Test Encoders", new TestEncoders());
		
		intake = new JoystickButton(right, 3);
		outtake = new JoystickButton(right, 5);
		openIntake = new JoystickButton(left, 6);
		closeIntake = new JoystickButton(left, 4);
		
		intake.whileHeld(new IntakeCube());
		outtake.whileHeld(new OuttakeCube());
		openIntake.whenPressed(new OpenIntake());
		closeIntake.whenPressed(new CloseIntake());
		
	}
	
	public Joystick getRightJoystick() {
		return right;
	}
	
	public double getRightX() {
		return right.getRawAxis(0);
	}
	
	public double getRightY() {
		return right.getRawAxis(1);
	}
	
	public double getRightZ() {
		if (right.getAxisCount() >= 3) {
			return right.getRawAxis(2);
		}
		System.out.println("Warning: Right joystick does not have 3 axes, but is being queried as if it did");
		return 0;
	}
	
	public Joystick getLeftJoystick() {
		return left;
	}
	
	public double getLeftX() {
		return left.getRawAxis(0);
	}
	
	public double getLeftY() {
		return left.getRawAxis(1);
	}
	
	public double getLeftZ() {
		if (left.getAxisCount() >= 3) {
			return left.getRawAxis(2);
		}
		System.out.println("Warning: Left joystick does not have 3 axes, but is being queried as if it did");
		return 0;
	}
}
