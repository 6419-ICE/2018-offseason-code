package org.usfirst.frc.team6419.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import org.usfirst.frc.team6419.robot.commands.HandleLift;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.VictorSP;

/**
 *
 */
public class Lift extends Subsystem {
	
	private VictorSP liftMotorController;
	
	public Lift(int liftMotorPin) {
		super("Lift");
		liftMotorController = new VictorSP(liftMotorPin);
		liftMotorController.setSubsystem(getSubsystem());
		liftMotorController.setName(getSubsystem(), "Lift Motor");
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new HandleLift());
    }
    
    public void stop() {
    	liftMotorController.set(0);
    }
    
    public void setPower(double power) {
    	liftMotorController.set(power);
    }
    
    public void park() {
    	liftMotorController.set(0.19);
    }
    /*
    @Override
    public void initSendable(SendableBuilder builder) {
    	builder.addDoubleProperty("Current", () -> liftMotorController.getOutputCurrent(), null);
    	super.initSendable(builder);
    }*/
}

