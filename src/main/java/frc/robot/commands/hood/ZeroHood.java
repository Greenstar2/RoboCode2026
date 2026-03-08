package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class ZeroHood extends Command
{
    public ZeroHood()
    {
        addRequirements(Hood.getInstance());
    }

    @Override
    public void initialize()
    {
        Hood.getInstance().setVoltage(Volts.of(Constants.Hood.ZEROING_VOLTAGE));
    }

    @Override
    public boolean isFinished()
    {
        return Hood.getInstance().isStalling();
    }

    @Override
    public void end(boolean interrupted)
    {
        if (!interrupted)
        {
            System.out.println("Soft zero done");
            Hood.getInstance().setPosition(Degrees.of(-0.5)); 
            // this is because the hood pops a bit after the current zero, so don't
            // want the hood to stall when it goes to 75.0 (or 0.0) degrees.
        }
        Hood.getInstance().setVoltage(Volts.of(0.0));
    }
}
