package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

    public class Drivetrain extends SubsystemBase {
     public CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_MOTOR, MotorType.kBrushless);
     public CANSparkMax leftRearMotor = new CANSparkMax(Constants.LEFT_REAR_MOTOR, MotorType.kBrushless);
     public CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
     public CANSparkMax rightRearMotor = new CANSparkMax(Constants.RIGHT_REAR_MOTOR, MotorType.kBrushless);

     public PigeonIMU gyro = new PigeonIMU(RobotContainer.indexer.backWheelMotor);

    //  DifferentialDrive drivetrain = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

     DifferentialDriveOdometry driveOdometry; 

    public Drivetrain() {
     rightRearMotor.setInverted(true);
     rightFrontMotor.setInverted(true);
     leftRearMotor.setInverted(false);
     leftFrontMotor.setInverted(false);

     leftFrontMotor.setIdleMode(IdleMode.kBrake);
     leftRearMotor.setIdleMode(IdleMode.kBrake);
     rightFrontMotor.setIdleMode(IdleMode.kBrake);
     rightRearMotor.setIdleMode(IdleMode.kBrake);
        
     leftRearMotor.follow(leftFrontMotor);
     rightRearMotor.follow(rightFrontMotor);

     gyro.setFusedHeading(0);

     driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getFusedHeading()));
    }

    // drive using control sticks
    public void drive() {
     double leftY = MathUtil.applyDeadband(RobotContainer.driverPad.getLeftYAxis(), 0.05);
     double rightX = MathUtil.applyDeadband(RobotContainer.driverPad.getRightXAxis(), 0.05);
     double y = -1 * leftY * Constants.DRIVE_Y;
     double rot = rightX * Constants.DRIVE_ROT;
    
     // Calculated Outputs (Limits Output to 12V)
     double leftOutput = rot + y ;
     double rightOutput = y - rot;
    
     // Assigns Each Motor's Power
     leftFrontMotor.set(leftOutput);
     rightFrontMotor.set(rightOutput);
     leftRearMotor.set(leftOutput);
     rightRearMotor.set(rightOutput);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftFrontMotor.setVoltage(leftVolts);
        rightFrontMotor.setVoltage(rightVolts);
        // drivetrain.feed();
      }

    // REV Neo integrated encoder's native units are rotations.
    public double calculateWheelDistanceMeters(double encoderRot) {
        double x = encoderRot;
        x /= Constants.kEncoderRotationToWheelRotationRatio;
        x *= Constants.kWheelDiameterInches * Math.PI;
        return Units.inchesToMeters(x);
    }

    // REV Neo integrated encoder's native units are RPM
    public double calculateWheelSpeedsMPS(double encoderRPM) {
        double x = encoderRPM;
        x /= Constants.kEncoderRotationToWheelRotationRatio;
        x *= Constants.kWheelDiameterInches * Math.PI;
        x /= 60;
        return Units.inchesToMeters(x);
    }

    public void resetEncoders() {
        leftFrontMotor.getEncoder().setPosition(0);
        rightFrontMotor.getEncoder().setPosition(0);
    }

    public double getHeading() {
        return gyro.getFusedHeading() % 360;
    }

    public Pose2d getPose() {
       return driveOdometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            calculateWheelSpeedsMPS(leftFrontMotor.getEncoder().getVelocity()),
            calculateWheelSpeedsMPS(rightFrontMotor.getEncoder().getVelocity()));
      }
    
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
      }

    // Timing based auto methods

    public void setDriver(double leftSpeed, double rightSpeed) {
        leftFrontMotor.set(leftSpeed);
        rightFrontMotor.set(rightSpeed);
    }

    public void setRotation(double angle) {
        double input = MathUtil.clamp(((gyro.getFusedHeading() - angle) * Constants.AUTO_DRIVE_ROTATE_KP), -0.7, 0.7);
        leftFrontMotor.set(input);
        rightFrontMotor.set(input * -1);
    }

    public void periodic() {
     if (DriverStation.isAutonomous()) {
         driveOdometry.update(Rotation2d.fromDegrees(getHeading()),    
          calculateWheelDistanceMeters(leftFrontMotor.getEncoder().getPosition()), 
          calculateWheelDistanceMeters(rightFrontMotor.getEncoder().getPosition()));
     }

     drive();

     SmartDashboard.putNumber("left drive encoder", leftFrontMotor.getEncoder().getPosition());
     SmartDashboard.putNumber("right drive encoder", rightFrontMotor.getEncoder().getPosition());
    }    
}
