package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robocats.controllers.Controller;
import frc.robot.subsystems.other.Elevator;

public class moveElevator extends Command {
    private Controller  grimyController;
    private int height = 0;

    Elevator subsystem;
    //called once when object is created
    // does not run on button press only when the robot starts
    public moveElevator(Elevator subsystem,Controller grimyController) {
        this.grimyController = grimyController; addRequirements(subsystem);
    }

    // runs once when the command is called. like pressing a button
    @Override
    public void initialize() {
       
    }

    // called over and over again
    // like a while (true) loop
    @Override
    public void execute() {
        subsystem.elevatorUp();
    }  

    // called once after isFinished returns true
    // interrupted is true when another command uses the same subsystem
    @Override
    public void end(boolean interrupted) {
            subsystem.elevatorStop();
        
    }

    // returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
        //if (height == 1 || height == 2 || height == 3 || height == 4 ) {
          //  return true;
       // }
       // else {
            //return false;
        //}
    }
}
