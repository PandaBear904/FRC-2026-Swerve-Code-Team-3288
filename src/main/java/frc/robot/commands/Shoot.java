package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystemCTRE;

public class Shoot extends Command {
    private final ShooterSubsystemCTRE shooter;
    private final double rpm;

    public Shoot(ShooterSubsystemCTRE shooter, double rpm){
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        shooter.setRPM(rpm);
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
