package frc.robot.subsystems.feeder;

import frc.robot.subsystems.feeder.FeederSubsystem.FeederState;

public interface IFeederListener {
    public abstract void handleFeederChange(FeederState state, FeederSubsystem feeder);

    public abstract void onFeederRegister(FeederSubsystem feeder);
}
