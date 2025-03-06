package frc.robot.State;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.PositionTracker;

public class StateMachine {
    private static Map<StateMachineStateName, ArrayList<StateMachineState>> m_stateMap = new HashMap<StateMachineStateName, ArrayList<StateMachineState>>();
    private static StateMachineStateName m_currentStateName = StateMachineStateName.Intake;

    private static Map<StateMachineStateName, StateMachineState> m_allStates = new HashMap<StateMachineStateName, StateMachineState>();

    public static void Load() {
        // states
        var intakeState = new StateMachineState(
                StateMachineStateName.Intake,
                Color.kRed,
                true,
                false,
                false,
                true);
        var postIntakeState = new StateMachineState(
                StateMachineStateName.PostIntake,
                Color.kBlue,
                true,
                false,
                false,
                false);
        var prepareScoreState = new StateMachineState(
                StateMachineStateName.PrepareScore,
                Color.kOrange,
                false,
                true,
                false,
                false);
        var scoreState = new StateMachineState(
                StateMachineStateName.Score,
                Color.kPurple,
                false,
                true,
                false,
                false);
        var preIntakeState = new StateMachineState(
                StateMachineStateName.PreIntake,
                Color.kGreen,
                false,
                false,
                false,
                true);

        m_allStates.put(intakeState.Name, intakeState);
        m_allStates.put(postIntakeState.Name, postIntakeState);
        m_allStates.put(prepareScoreState.Name, prepareScoreState);
        m_allStates.put(scoreState.Name, scoreState);
        m_allStates.put(preIntakeState.Name, preIntakeState);

        // transitions
        var intakeList = new ArrayList<StateMachineState>();
        intakeList.add(postIntakeState);
        intakeList.add(preIntakeState);
        m_stateMap.put(intakeState.Name, intakeList);

        var postIntakeList = new ArrayList<StateMachineState>();
        postIntakeList.add(preIntakeState);
        postIntakeList.add(prepareScoreState);
        m_stateMap.put(postIntakeState.Name, postIntakeList);

        var prepareScoreList = new ArrayList<StateMachineState>();
        prepareScoreList.add(prepareScoreState);
        prepareScoreList.add(scoreState);
        prepareScoreList.add(preIntakeState);
        m_stateMap.put(prepareScoreState.Name, prepareScoreList);

        var scoreList = new ArrayList<StateMachineState>();
        scoreList.add(prepareScoreState);
        scoreList.add(preIntakeState);
        m_stateMap.put(scoreState.Name, scoreList);

        var preIntakeList = new ArrayList<StateMachineState>();
        preIntakeList.add(intakeState);
        preIntakeList.add(postIntakeState);
        m_stateMap.put(preIntakeState.Name, preIntakeList);
    }

    public static StateMachineState GetCurrentState() {
        return m_allStates.get(m_currentStateName);
    }

    public static Boolean CanTransition(
            PositionTracker positionTracker,
            StateMachineStateName toStateName) {
        System.out.println("From " + m_currentStateName.toString() + " to " + toStateName.toString());

        // look in map
        var toStates = m_stateMap.get(m_currentStateName);

        // from state not found
        if (toStates == null || toStates.isEmpty()) {
            System.out.println("Failed: from state not found");
            return false;
        }

        // to state not found
        StateMachineState toState = null;
        for (var state : toStates) {
            if (state.Name == toStateName) {
                toState = state;
                break;
            }
        }
        if (toState == null) {
            System.out.println("Failed: to state not found");
            return false;
        }

        // check booleans
        if (toState.RequiresIntakeCoral && !positionTracker.getCoralInTray()) {
            System.out.println("Failed: coral not in tray");
            return false;
        }
        if (toState.RequiresArmCoral && !positionTracker.getCoralInArm()) {
            System.out.println("Failed: coral not in arm");
            return false;
        }
        if (toState.NotIfIntakeCoral && positionTracker.getCoralInTray()) {
            System.out.println("Failed: coral in tray");
            return false;
        }
        if (toState.NotIfArmCoral && positionTracker.getCoralInArm()) {
            System.out.println("Failed: coral in arm");
            return false;
        }

        // can transition
        m_currentStateName = toStateName;
        return true;
    }
}
