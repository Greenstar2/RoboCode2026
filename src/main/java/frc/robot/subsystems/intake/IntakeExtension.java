package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;

public class IntakeExtension extends SubsystemBase 
{
    private static IntakeExtension instance;
    private TalonFX motor;

    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.IntakeExtension.GEAR_RATIO),
            DCMotor.getKrakenX60(1), Constants.IntakeExtension.MIN_HEIGHT, Constants.IntakeExtension.MAX_HEIGHT, false, Constants.IntakeExtension.MIN_HEIGHT);

    private Debouncer stallingDebouncer;
    
    private IntakeExtension()
    {
        motor = new TalonFX(Constants.IntakeExtension.ID, Constants.CAN_CHAIN);
        config();
        
        if (isSimulated())
        {
            motor.getSimState().Orientation = Constants.IntakeExtension.MECHANICAL_ORIENTATION;
            motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }

        stallingDebouncer = new Debouncer(Constants.IntakeExtension.STALLING_DEBOUNCE_TIME);
    }

    private void config()
    {
        motor.clearStickyFaults();
        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();

        extensionConfig.Feedback.SensorToMechanismRatio = Constants.IntakeExtension.GEAR_RATIO;

        extensionConfig.MotorOutput.Inverted = Constants.IntakeExtension.INVERTED;
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        extensionConfig.Slot0.kP = Constants.IntakeExtension.KP;
        extensionConfig.Slot0.kI = Constants.IntakeExtension.KI;
        extensionConfig.Slot0.kD = Constants.IntakeExtension.KD;

        extensionConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        extensionConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.IntakeExtension.STATOR_CURRENT_LIMIT;
        extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.IntakeExtension.SUPPLY_CURRENT_LIMIT;

        motor.getConfigurator().apply(extensionConfig);
    }
    
    public Voltage getVoltage()
    {
        return motor.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getVelocity()
    {
        return motor.getVelocity().getValue();
    }

    public Current getStatorCurrent()
    {
        return motor.getStatorCurrent().getValue();
    }
   
    public void setVoltage (Voltage voltage)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to IntakeExtension");
            return;
        }
        motor.setControl(new VoltageOut(voltage));
    }

    public boolean isStalling()
    {
        if (getVoltage().in(Volts) > 0)
        {
            return stallingDebouncer.calculate(motor.getStatorCurrent().getValueAsDouble() >= Constants.IntakeExtension.STALLING_CURRENT_EXTEND);
        }
        else
        {
            return stallingDebouncer.calculate(motor.getStatorCurrent().getValueAsDouble() >= Constants.IntakeExtension.STALLING_CURRENT_RETRACT);
        }
    }
    
    @Override
    public void periodic ()
    {
        if (isSimulated())
        {
            TalonFXSimState simState = motor.getSimState();

            simState.setSupplyVoltage(RobotController.getBatteryVoltage());
            Voltage elevatorMotorVoltage = simState.getMotorVoltageMeasure();

            sim.setInputVoltage(elevatorMotorVoltage.in(Volts));
            sim.update(0.020);

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            simState.setRawRotorPosition(sim.getPositionMeters() * Constants.IntakeExtension.GEAR_RATIO);
            simState.setRotorVelocity(sim.getVelocityMetersPerSecond() * Constants.IntakeExtension.GEAR_RATIO);
        }
    }
    
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INTAKE_EXTENSION_INDEX) == SubsystemStatus.Simulated;
    }
    
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INTAKE_EXTENSION_INDEX) == SubsystemStatus.Disabled;
    }

    public static IntakeExtension getInstance()
    {
        if(instance == null) instance = new IntakeExtension();
        return instance;
    }
}
