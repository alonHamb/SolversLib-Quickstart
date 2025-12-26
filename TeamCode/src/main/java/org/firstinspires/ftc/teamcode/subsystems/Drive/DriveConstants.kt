package org.firstinspires.ftc.teamcode.subsystems.Drive

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.hamosad1657.lib.math.PIDGains
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.meters

object DriveConstants {
    val DRIVE_MOTOR_TYPE = Motor.GoBILDA.RPM_312

    val WHEEL_RADIUS = 0.104.meters

    val TRACK_WIDTH = 0.4572.meters

    object Roadrunner {

        const val K_V = 0.0

        const val K_A = 0.0

        const val K_STATIC = 0.0

        const val RUN_USING_ENCODER = true

        val MOTOR_VELOCITY_PID_GAINS = PIDGains(0.0, 0.0, 0.0, { 0.0 })

        val ROBOT_MOVEMENT_PID_GAINS = PIDCoefficients(0.0, 0.0, 0.0)

        val ROBOT_HEADING_PID_GAINS = PIDCoefficients(0.0, 0.0, 0.0)

        val ROBOT_PATH_FOLLOWER_DISTANCE_TOLERANCE = Vector2d(0.1.meters.asMeters, 0.1.meters.asMeters)

        val ROBOT_PATH_FOLLOWER_ANGLE_TOLERANCE = 5.degrees

    }


    object Distance {

        val FRONT_LEFT_DISTANCE_FROM_CENTER: Translation2d = Translation2d(-0.20112, 0.20112)
        val FRONT_RIGHT_DISTANCE_FROM_CENTER: Translation2d = Translation2d(0.20112, 0.20112)
        val BACK_LEFT_DISTANCE_FROM_CENTER: Translation2d = Translation2d(-0.20112, -0.20112)
        val BACK_RIGHT_DISTANCE_FROM_CENTER: Translation2d = Translation2d(0.20112, -0.20112)

    }
}