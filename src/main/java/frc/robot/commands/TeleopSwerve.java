package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.RotationCenter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier slowSpeed;
    private IntSupplier centerOfRotationSup;
    private Swerve.RotationCenter center;

    public TeleopSwerve(
            Swerve s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            BooleanSupplier slowSpeed,
            IntSupplier centerOfRotationSup
            ) {

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowSpeed = slowSpeed;
        this.centerOfRotationSup = centerOfRotationSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), OIConstants.translationDeadBand);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), OIConstants.translationDeadBand);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), OIConstants.turnDeadBand);

        switch (centerOfRotationSup.getAsInt()) {
            case -1:
                center = RotationCenter.CENTER;
                break;
            case 0:
                center = RotationCenter.FRONTLEFT;
                break;
            case 1:
                center = RotationCenter.FRONTRIGHT;
                break;
            case 2:
                center = RotationCenter.BACKLEFT;
                break;
            case 3:
                center = RotationCenter.BACKRIGHT;
                break;
            default:
                center = RotationCenter.CENTER;

        }

        s_Swerve.setCenterOfRotation(center);
        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).
                    times(DrivetrainConstants.maxSpeed).
                    times(slowSpeed.getAsBoolean()?OIConstants.lowTranslationSpeed: OIConstants.highTranslationSpeed),
                rotationVal * DrivetrainConstants.maxAngularVelocity * (slowSpeed.getAsBoolean()? OIConstants.lowRotationSpeed: OIConstants.highRotationSpeed),
                !robotCentricSup.getAsBoolean(),
                true);
    }
}