package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.ShooterState
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants.ShooterState.Companion.AT_GOAL
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem as Shooter

fun Shooter.getToStateCommand(state: ShooterState): Command = RunCommand({ this.setShooterState(state) }, this)

fun Shooter.getToGoalStateCommand(): Command = RunCommand({ this.setShooterState(AT_GOAL) })

fun Shooter.followGoalCommand(): Command = RunCommand({ shooter.followGoal() })

