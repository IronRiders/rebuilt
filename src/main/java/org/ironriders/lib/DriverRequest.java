package org.ironriders.lib;

import org.ironriders.core.TargetingControl;

import dev.doglog.DogLog;

public class DriverRequest {
    public enum PriorityMode {
        DRIVER_PRIORITY(),
        LAUNCHER_PRIORITY(),
        ALIGN_PRIORITY();
    }

    public enum AlignTargetingMode {
        LAUNCHER(),
        OUTPOST(),
        BUMP();
    }

    public enum LauncherTargetingMode {
        HUB(),
        PASSING();
    }

    public PriorityMode r_priorityMode = PriorityMode.DRIVER_PRIORITY;
    public AlignTargetingMode r_alignTargetingMode = AlignTargetingMode.LAUNCHER;
    public LauncherTargetingMode r_launcherTargetingMode = LauncherTargetingMode.HUB;

    public DriverRequest(PriorityMode priorityMode) {
        this.r_priorityMode = priorityMode;
    }

    public DriverRequest(PriorityMode priorityMode, AlignTargetingMode alignTargetingMode) {
        this.r_priorityMode = priorityMode;
        this.r_alignTargetingMode = alignTargetingMode;
    }

    public DriverRequest(PriorityMode priorityMode, LauncherTargetingMode launcherTargetingMode) {
        this.r_priorityMode = priorityMode;
        this.r_launcherTargetingMode = launcherTargetingMode;
    }

    public DriverRequest(PriorityMode priorityMode, AlignTargetingMode alignTargetingMode,
            LauncherTargetingMode launcherTargetingMode) {
        this.r_priorityMode = priorityMode;
        this.r_alignTargetingMode = alignTargetingMode;
        this.r_launcherTargetingMode = launcherTargetingMode;
    }

    public void send(String debugName) {
        TargetingControl.receiveRequest(this);
        DogLog.log("Sent", "Name: " + debugName + "TargetingMode: " + this.r_alignTargetingMode.toString() + " LauncherMode: "
                + this.r_launcherTargetingMode.toString() + " AlignMode: " + this.r_alignTargetingMode.toString());
    }
}