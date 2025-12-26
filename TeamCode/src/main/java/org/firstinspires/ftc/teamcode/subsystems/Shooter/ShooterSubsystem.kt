package org.firstinspires.ftc.teamcode.subsystems.Shooter

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.motors.HaDcMotor
import org.firstinspires.ftc.teamcode.alonlib.servos.HaCrServo
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.div
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.ANGLE_SERVO_RATIO
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.FLYWHEEL_MOTOR_PID_GAINS
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.FLYWHEEL_MOTOR_RATIO
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.FLYWHEEL_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.HEADING_SERVO_RATIO
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.HEADING_TOLERANCE
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.HOOD_SERVO_MAX_POSITION
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.HOOD_SERVO_MIN_POSITION
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.ShooterState
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.TURRET_ENCODER_RANGE
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.TURRET_ENCODER_UNIT
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.TURRET_SERVO_RUN_MODE
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.VELOCITY_TOLERANCE
import org.firstinspires.ftc.teamcode.RobotMap.Shooter as ShooterMap

class ShooterSubsystem(hardwareMap: HardwareMap, telemetry: Telemetry, alliance: Alliance) : SubsystemBase() {
    init {
        name = "Shooter Subsystem"

    }

    val hoodServo = HaServo(
        hardwareMap,
        ShooterMap.HOOD_SERVO_ID,
        HOOD_SERVO_MIN_POSITION,
        HOOD_SERVO_MAX_POSITION
    )

    val turretEncoder = AbsoluteAnalogEncoder(
        hardwareMap,
        ShooterMap.TURRET_ENCODER_ID,
        TURRET_ENCODER_RANGE.degrees,
        TURRET_ENCODER_UNIT
    )

    val turretServo = HaCrServo(
        hardwareMap,
        ShooterMap.TURRET_SERVO_ID,
        TURRET_SERVO_RUN_MODE
    ).apply {
        setAbsoluteEncoder(turretEncoder)
        setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)
    }

    val flywheelMotor = HaDcMotor(
        hardwareMap,
        ShooterMap.FLYWHEEL_MOTOR_ID,
        FLYWHEEL_MOTOR_TYPE
    ).apply {
        setRunMode(Motor.RunMode.VelocityControl)
        setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)
        setVeloCoefficients(
            FLYWHEEL_MOTOR_PID_GAINS.kP,
            FLYWHEEL_MOTOR_PID_GAINS.kI,
            FLYWHEEL_MOTOR_PID_GAINS.kD
        )
    }

    // --- State Getters ---

    val currentState: ShooterState get() = ShooterState(currentHeading, currentAngle, currentVelocity)

    val currentVelocity: AngularVelocity get() = flywheelMotor.getCurrentVelocity() * FLYWHEEL_MOTOR_RATIO


    /** the [currentHeading] of the turret relative to the robot **/
    val currentHeading: Rotation2d get() = (turretServo.encoder.position / turretServo.cpr).degrees * HEADING_SERVO_RATIO
    val currentAngle: Rotation2d get() = (hoodServo.getPositionSetPoint().degrees * ANGLE_SERVO_RATIO).degrees * ANGLE_SERVO_RATIO


    val headingError: Rotation2d get() = turretServo.targetPosition - turretEncoder.currentPosition.degrees

    val velocityError: AngularVelocity get() = flywheelMotor.targetVelocity - flywheelMotor.getCurrentVelocity()

    val isWithinVelocityTolerance: Boolean get() = VELOCITY_TOLERANCE >= velocityError

    val isWithinHeadingTolerance: Boolean get() = HEADING_TOLERANCE >= headingError

    fun setHoodAngle(angle: Rotation2d) {
        hoodServo.setPositionSetPoint(angle / ANGLE_SERVO_RATIO)
    }

    fun setHeading(heading: Rotation2d) {
        turretServo.setPositionSetPoint(heading / HEADING_SERVO_RATIO)
    }

    fun setFlywheelVelocity(velocity: AngularVelocity) {
        flywheelMotor.setVelocitySetpoint(velocity / FLYWHEEL_MOTOR_RATIO)
    }

    fun setShooterState(state: ShooterState) {
        setHeading(state.heading)
        setHoodAngle(state.angle)
        setFlywheelVelocity(state.velocity)
    }

    fun followGoal() {
        setHeading(vision.angleToGOalTag())
    }

    fun stop() {
        setHeading(0.degrees)
        setHoodAngle(0.degrees)
        setFlywheelVelocity(0.rpm)
    }

    fun disable() {
        flywheelMotor.disable()
        turretServo.disable()
        hoodServo.disable()
    }


}

