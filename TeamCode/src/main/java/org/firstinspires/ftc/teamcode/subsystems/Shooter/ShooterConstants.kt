package org.firstinspires.ftc.teamcode.subsystems.Shooter

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.rpm
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

object ShooterConstants {
    // --- configs ---
    val HOOD_SERVO_MIN_POSITION = (-150).degrees
    val HOOD_SERVO_MAX_POSITION = 150.degrees

    val TURRET_SERVO_RUNMODE = CRServoEx.RunMode.OptimizedPositionalControl

    val TURRET_ENCODER_RANGE = 360.degrees
    val TURRET_ENCODER_UNIT = AngleUnit.DEGREES

    val FLYWHEEL_MOTOR_TYPE = Motor.GoBILDA.BARE
    val FLYWHEEL_MOTOR_PID_GAINS: PIDGains = PIDGains(0.0,0.0,0.0,)


    // --- Ratios ---
    const val HOOD_TO_SERVO_RATIO = 1.0

    // --- Constants ---
    val MAX_HOOD_ANGLE = 82.degrees
    val MIN_HOOD_ANGLE = 45.degrees
    val MAX_FLYWHEEL_VELOCITY = 6000.rpm

    class ShooterState(heading: Rotation2d, angle: Rotation2d, velocity: AngularVelocity) {
        val heading: Rotation2d
        val angle: Rotation2d
        val velocity: AngularVelocity

        init {
            this.angle =
                if (angle.degrees in MIN_HOOD_ANGLE.degrees..MAX_HOOD_ANGLE.degrees) angle
                else {
                    robotPrintError("hood angle out of bounds: ${angle.degrees}")
                    0.degrees
                }
            this.velocity =
                if (velocity.asRpm in 0.0..MAX_FLYWHEEL_VELOCITY.asRpm) velocity
                else {
                    robotPrintError("flywheel speed out of bounds: ${velocity.asRpm}")
                    0.rpm
                }
            this.heading = heading
        }

        companion object {

            val AT_GOAL = ShooterState(0.degrees, 82.degrees, 3000.rpm)

            val AT_FAR_STARTING_ZONE = ShooterState(45.degrees, 45.degrees, 6000.rpm)

        }
    }
}