package org.firstinspires.ftc.teamcode.subsystems.Shooter

import com.hamosad1657.lib.units.AngularVelocity
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.alonlib.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.motors.HaDcMotor
import org.firstinspires.ftc.teamcode.alonlib.servos.HaCrServo
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.FLYWHEEL_MOTOR_PID_GAINS
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.FLYWHEEL_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.HOOD_SERVO_MAX_POSITION
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.HOOD_SERVO_MIN_POSITION
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.HOOD_TO_SERVO_RATIO
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.ShooterState
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.TURRET_ENCODER_RANGE
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.TURRET_ENCODER_UNIT
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.TURRET_SERVO_RUNMODE
import org.firstinspires.ftc.teamcode.RobotMap.Shooter as ShooterMap

class ShooterSubsystem(hardwareMap: HardwareMap, telemetry: Telemetry) : SubsystemBase() {
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
        ShooterMap.TURRENT_SERVO_ID,
        TURRET_SERVO_RUNMODE
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

    val currntVelocity get() = AngularVelocity.fromRpm(flywheelMotor.velocity)

    fun setHoodAngle(angle: Rotation2d) {
        hoodServo.setPosition(angle * HOOD_TO_SERVO_RATIO)
    }

    fun setHeading(heading: Rotation2d) {
        turretServo.setPositionSetPoint(heading)
    }

    fun setFlywheelVelocity(velocity: AngularVelocity) {
        flywheelMotor.setVelocitySetpoint(velocity)
    }

    fun setShooterState(state: ShooterState) {
        setHeading(state.heading)
        setHoodAngle(state.angle)
        setFlywheelVelocity(state.velocity)
    }

    fun disable() {
        flywheelMotor.disable()
        turretServo.disable()
        hoodServo.disable()
    }


}