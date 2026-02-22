package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.ClimbConstants;
import swervelib.parser.json.MotorConfigDouble;
import frc.robot.Constants.CanbusName;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climb extends SubsystemBase{
    //devices
    private final TalonFX m_climb = new TalonFX(ClimbConstants.kClimbMotor,  CanbusName.rioCANBus);
    private final CANcoder cancoder;

    //configs
    private final TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_climbCurrentConfig = new CurrentLimitsConfigs();

    private final FeedbackConfigs feedback = new FeedbackConfigs(); //what is ts

    //moton magic
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
    private double position;

    public Climb(){
        m_climb.stopMotor();
        cancoder = new CANcoder(ClimbConstants.kClimbCANcoder, CanbusName.rioCANBus);

        //PID values
        climbConfig.Slot0.kS = ClimbConstants.kClimbControllerS;
        climbConfig.Slot0.kV = ClimbConstants.kClimbControllerV;
        climbConfig.Slot0.kA = ClimbConstants.kClimbControllerA;
        climbConfig.Slot0.kP = ClimbConstants.kClimbControllerP;
        climbConfig.Slot0.kI = ClimbConstants.kClimbControllerI;
        climbConfig.Slot0.kD = ClimbConstants.kClimbControllerD;

        //Motion Magic settings-- again do i need this? og code has it and it seems helpful
        climbConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.kMotionMagicCruiseVelocity;
        climbConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.kMotionMagicAcceleration;
        climbConfig.MotionMagic.MotionMagicJerk = ClimbConstants.kMotionMagicJerk;

        //current limit confih
        m_climbCurrentConfig.withStatorCurrentLimit(ClimbConstants.kClimbCurrentStatorLimit);
        m_climbCurrentConfig.withSupplyCurrentLimit(ClimbConstants.kClimbCurrentSupplyLimit);

        //cancoder config??
        feedback.FeedbackRemoteSensorID = ClimbConstants.kClimbCANcoder;
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        //config applincation???
        climbConfig.withFeedback(feedback);
        climbConfig.withCurrentLimits(m_climbCurrentConfig);
        m_climb.getConfigurator().apply(climbConfig);
        m_climb.setNeutralMode(NeutralModeValue.Brake);//brake mode
        //m_arm.setSafetyEnabled(true);//Turns on safety i love being unsafe

    
    };

    public void setArmPosition(double armPosition){
    position = armPosition * ClimbConstants.kClimbCANCoderConversionFactor;
    System.out.println("Processed armPos: " + position);
    m_climb.setControl(motionMagicControl.withPosition(position));
    }

    public void climbMotorStop(){
        m_climb.stopMotor();
    }

    public void climbUp(){
        m_climb.set(ClimbConstants.kClimbUpSpeed);
    }

    public void climbDown(){
        m_climb.set(ClimbConstants.kClimbDownSpeed);
    }

    //will set full up position to encoder's "zero"
    public void climbZeroEncoder(){
        //waht do i even put in here man
    }

    //lawrence pls dont execute me im trying my best
}

