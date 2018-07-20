package org.usfirst.frc.team6419.robot.commands.auto;

import org.usfirst.frc.team6419.robot.commands.ResetGyro;
import org.usfirst.frc.team6419.robot.commands.core.DriveToPoint;
import org.usfirst.frc.team6419.robot.commands.core.TurnToHeading;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SwitchAuto extends CommandGroup {

    public SwitchAuto() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
    	addSequential(new ResetGyro());
    	addSequential(new DriveToPoint(0, 60));
    	if (DriverStation.getInstance().getGameSpecificMessage().startsWith("L")) {
    		addSequential(new DriveToPoint(60, 0));
    		addSequential(new TurnToHeading(-0.5 * Math.PI));
    	} else {
    		addSequential(new DriveToPoint(-60, 0));
    		addSequential(new TurnToHeading(0.5 * Math.PI));
    	}
    }
}
