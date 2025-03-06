package frc.robot.State;

import edu.wpi.first.wpilibj.util.Color;

public class StateMachineState {
    public StateMachineStateName Name;
    public Color Color = null;
    public Boolean RequiresIntakeCoral = false;
    public Boolean RequiresArmCoral = false;
    public Boolean NotIfIntakeCoral = false;
    public Boolean NotIfArmCoral = false;

    public StateMachineState(
            StateMachineStateName name,
            Color color,
            Boolean requiresIntakeCoral,
            Boolean requiresArmCoral,
            Boolean notIfIntakeCoral,
            Boolean notIfArmCoral) {
        super();

        Name = name;
        Color = color;
        RequiresIntakeCoral = requiresIntakeCoral;
        RequiresArmCoral = requiresArmCoral;
        NotIfIntakeCoral = notIfIntakeCoral;
        NotIfArmCoral = notIfArmCoral;
    }
}
