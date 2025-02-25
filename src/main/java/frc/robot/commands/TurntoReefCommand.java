package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TurntoReefCommand extends Command {
    private final Swerve s_Swerve;
    private final Limelight s_Limelight;
    private boolean complete = false;
    private double angle;
    private Timer timer = new Timer();
    private double timout;
    
    private Map<Integer, Integer> angles = new HashMap<Integer, Integer>(){{
        put(6, 120);
        put(7, 180);
        put(8, 240);
        put(9, 300);
        put(10, 0);
        put(11, 60);
        put(17, 60);
        put(18, 0);
        put(19, 300);
        put(20, 240);
        put(21, 180);
        put(22, 120);
    }};

public TurntoReefCommand (Swerve subsystem, Limelight limeLight, double timeoutS){
    s_Swerve = subsystem;
    s_Limelight = limeLight;
    timout = timeoutS;
    addRequirements(subsystem);
    addRequirements(limeLight);
}
public void initialize() {
    timer.reset();
    timer.reset();
    complete = false;
    int tag = s_Limelight.aprilTagID();
    boolean exists = angles.containsKey(tag);
    if(exists){
        int getAngle = angles.get(tag);
        angle = getAngle;
    }
    else{
        complete = true;
    }
}
}