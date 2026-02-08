package org.ironriders.core;

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

    PriorityMode r_priorityMode;
    AlignTargetingMode r_alignTargetingMode;
    LauncherTargetingMode r_launcherTargetingMode;

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

    public void send() {
        TargetingControl.receiveRequest(this);
    }
}