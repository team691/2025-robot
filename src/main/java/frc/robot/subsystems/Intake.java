/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.floorIntake;

import com.ctre.phoenix6.hardware.*;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

// used for "output" in order to throw note into goal

// TODO: Potentially use limelight to only allow shooting of the ring IF limelight is aligned with the target (requires offsets and knowing where it'll be placed)

public class Intake extends SubsystemBase{

    // Motors
    

    // Speaker motors
    private final TalonFX motor13 = new TalonFX(floorIntake.id14);
    private final TalonFX motor14 = new TalonFX(floorIntake.id15);
    // Release motor
    private final CANSparkMax motor15 = new CANSparkMax(floorIntake.id16, MotorType.kBrushless);

    // Initialize new output
    public Intake() {

        // By default, motors will be stopped
        motor13.setNeutralMode(NeutralModeValue.Brake);
        motor14.setNeutralMode(NeutralModeValue.Brake);
        motor15.setIdleMode(IdleMode.kBrake);
    }

    public void periodic() {
        // called periodically
    }

    public Command IntakeRing() {
        return run(
            () -> {
                motor13.set(3);
                motor14.set(3);
            });
    }
    
    public Command stopRun() {
        return run(
            () -> {
                motor13.set(0);
                motor14.set(0);
                motor15.set(0);
            });
        }
}
*/