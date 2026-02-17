package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;

public class Agitator extends Command{
    private final AgitatorSubsystem agitator;
    private final double volts;

    public Agitator(AgitatorSubsystem agitator, double volts){
        this.agitator = agitator;
        this.volts = volts;
        addRequirements(agitator);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        agitator.setVoltage(volts);
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
