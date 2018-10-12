package org.usfirst.frc.team6419.robot.commands.auto;

import org.usfirst.frc.team6419.robot.Robot;
import org.usfirst.frc.team6419.robot.commands.OpenIntake;
import org.usfirst.frc.team6419.robot.commands.OuttakeCube;
import org.usfirst.frc.team6419.robot.commands.core.DriveToPoint;
import org.usfirst.frc.team6419.robot.commands.core.RaiseLiftForTime;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SwitchAuto2WithDrive2Point extends CommandGroup {

    public SwitchAuto2WithDrive2Point() {
    	addParallel(new RaiseLiftForTime(1.55555));
    	//Robot.drivetrain.setSpeedLimit(0.5);
    	if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
    		Robot.log(this, "AddParallel");
    		// Straight shot
    		addSequential(new DriveToPoint(0, 104));
    	} else {
    		Robot.log(this, "Step 1");
    		addSequential(new DriveToPoint(0, 32));
    		Robot.log(this, "Step 2");
    		addSequential(new DriveToPoint(-108, 0));
    		Robot.log(this, "Step 3");
    		addSequential(new DriveToPoint(72, 0));
    	}
    	//Robot.drivetrain.setSpeedLimit(1);
    	addSequential(new OpenIntake());
    	
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
    }
}
