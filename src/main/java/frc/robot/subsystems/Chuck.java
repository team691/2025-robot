package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChuckConstants;

import com.ctre.phoenix6.hardware.*;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

// used for "output" in order to throw note into goal

// TODO: Potentially use limelight to only allow shooting of the ring IF limelight is aligned with the target (requires offsets and knowing where it'll be placed)

public class Chuck extends SubsystemBase{

    // Motors
    

    // Speaker motors
    private final TalonFX motor10 = new TalonFX(ChuckConstants.id10);
    private final TalonFX motor11 = new TalonFX(ChuckConstants.id11);
    // Release motor
    private final CANSparkMax motor12 = new CANSparkMax(ChuckConstants.id12, MotorType.kBrushless);

    // Initialize new output
    public Chuck() {

        // By default, motors will be stopped
        motor10.setNeutralMode(NeutralModeValue.Brake);
        motor11.setNeutralMode(NeutralModeValue.Brake);
        motor12.setIdleMode(IdleMode.kBrake);
    }

    public void periodic() {
        // called periodically
    }

    public Command IntakeRing() {
        return run(
            () -> {
                motor10.set(3);
                motor11.set(3);
            });
    }

    public Command SpeakerShoot() {
        return run(
            () -> {
                motor10.set(-20);
                motor11.set(-20);
            });
    }

    public Command Outake() {
        return run(
            () -> {
                motor12.set(10);
            });
    }
    
    public Command stopRun() {
        return run(
            () -> {
                motor10.set(0);
                motor11.set(0);
                motor12.set(0);
            });
        }
        
    public Command stopRunAmp() {
        return run(
            () -> {
                motor12.set(0.0);
            });
    }
}
