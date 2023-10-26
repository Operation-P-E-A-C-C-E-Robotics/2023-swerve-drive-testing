package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private SlewRateLimiter velocityLimiter = new SlewRateLimiter(100);
    private SlewRateLimiter angleLimiter = new SlewRateLimiter(100);
    private double deadband = 0.01;
    private double angleSmoothingRange = 0.2;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        double rotationVal = rotationSup.getAsDouble();

        Translation2d translation = new Translation2d(translationVal, strafeVal);
        double linearVelocity = velocityLimiter.calculate(translation.getNorm());

        if(Math.abs(translation.getNorm()) < deadband){
            translation = new Translation2d(0, 0);
            velocityLimiter.reset(0);
        } else if (Math.abs(linearVelocity) < angleSmoothingRange){
            double angle = translation.getAngle().getRadians();
            double smoothedAngle = angleLimiter.calculate(angle);
            if(Math.abs(smoothedAngle - angle) > 0.1){
                angleLimiter.reset(angle);
                smoothedAngle = angle;
            }
            translation = new Translation2d(linearVelocity, smoothedAngle);
        }

        /* Drive */
        s_Swerve.drive(
            translation.times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}