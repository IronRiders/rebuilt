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

    public PriorityMode m_priorityMode = PriorityMode.DRIVER_PRIORITY;
    public AlignTargetingMode m_alignTargetingMode = AlignTargetingMode.LAUNCHER;
    public LauncherTargetingMode m_launcherTargetingMode = LauncherTargetingMode.HUB;

    public DriverRequest(PriorityMode priorityMode) {
        this.m_priorityMode = priorityMode;
    }

    public DriverRequest(PriorityMode priorityMode, AlignTargetingMode alignTargetingMode) {
        this.m_priorityMode = priorityMode;
        this.m_alignTargetingMode = alignTargetingMode;
    }

    public DriverRequest(PriorityMode priorityMode, LauncherTargetingMode launcherTargetingMode) {
        this.m_priorityMode = priorityMode;
        this.m_launcherTargetingMode = launcherTargetingMode;
    }

    public DriverRequest(PriorityMode priorityMode, AlignTargetingMode alignTargetingMode,
            LauncherTargetingMode launcherTargetingMode) {
        this.m_priorityMode = priorityMode;
        this.m_alignTargetingMode = alignTargetingMode;
        this.m_launcherTargetingMode = launcherTargetingMode;
    }

    public void send(String debugName) {
        TargetingControl.receiveRequest(this);
        DogLog.log("Sent", "Name: " + debugName + "TargetingMode: " + this.m_alignTargetingMode.toString() + " LauncherMode: "
                + this.m_launcherTargetingMode.toString() + " AlignMode: " + this.m_alignTargetingMode.toString());
    }
}