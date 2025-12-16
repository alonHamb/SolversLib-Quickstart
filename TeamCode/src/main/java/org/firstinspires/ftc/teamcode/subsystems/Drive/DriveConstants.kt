package org.firstinspires.ftc.teamcode.subsystems.Drive

import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.alonlib.units.meters

object DriveConstants {

    val WHEEL_RADIUS = 0.104.meters
    val FRONT_LEFT_MOTOR_TYPE = Motor.GoBILDA.RPM_312
    val FRONT_RIGHT_MOTOR_TYPE = Motor.GoBILDA.RPM_312
    val BACK_LEFT_MOTOR_TYPE = Motor.GoBILDA.RPM_312
    val BACK_RIGHT_MOTOR_TYPE = Motor.GoBILDA.RPM_312

    object Distance {

        val FRONT_LEFT_DISTANCE_FROM_CENTER: Translation2d = Translation2d(-0.20112, 0.20112)
        val FRONT_RIGHT_DISTANCE_FROM_CENTER: Translation2d = Translation2d(0.20112, 0.20112)
        val BACK_LEFT_DISTANCE_FROM_CENTER: Translation2d = Translation2d(-0.20112, -0.20112)
        val BACK_RIGHT_DISTANCE_FROM_CENTER: Translation2d = Translation2d(0.20112, -0.20112)

    }
}