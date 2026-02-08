package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsytem;

public class Shoot extends Command {
    private final ShooterSubsytem shooter;
    private final double volts;

    public Shoot(ShooterSubsytem shooter, double volts){
        this.shooter = shooter;
        this.volts = volts;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        shooter.setVoltage(volts);
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
