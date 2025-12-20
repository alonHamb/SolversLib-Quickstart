package org.firstinspires.ftc.teamcode.subsystems.vision

import com.seattlesolvers.solverslib.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.meters

object VisionConstants {

    val CAMERA_PITCH = 21.degrees

    val CAMERA_ROLL = 0.degrees

    val CAMERA_DISTANCE_FROM_TURRET_CENTER_AXIS_VECTOR = Translation2d((-0.13850).meters.asMeters, 0.0)

    val TURRET_CENTER_DISTANCE_FROM_ROBOT_CENTER_AXIS_VECTOR = Translation2d(0.07860.meters.asMeters, 0.0)

    val CAMERA_Z_HEIGHT_FROM_ORIGIN = 0.10378.meters

    object AprilTagsIDs {

        object Red {

            const val GOAL = 24

        }

        object Blue {

            const val GOAL = 20

        }

        object Obelisk {
            const val GPP = 21

            const val PGP = 22

            const val PPG = 23
        }


    }
}