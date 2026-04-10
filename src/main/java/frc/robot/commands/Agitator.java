package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;

public class Agitator extends Command{
    private final AgitatorSubsystem agitator;
    private final double rpm;

    public Agitator(AgitatorSubsystem agitator, double rpm){
        this.agitator = agitator;
        this.rpm = rpm;
        addRequirements(agitator);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        agitator.setRPM(rpm);
    }

    @Override
    public void end(boolean interrupted){
        agitator.stopAgitator();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
