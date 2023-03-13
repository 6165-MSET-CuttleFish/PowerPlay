package org.firstinspires.ftc.teamcode.util.moduleUtil;

public class Task
{
    public HwModule referenceModule;
    public ModuleState referenceState;
    public long delayTime=0;
    public RunCondition condition =new RunCondition(()->true);


    public Task(HwModule module, ModuleState state)
    {
        referenceModule=module;
        referenceState=state;
    }

    public Task(HwModule module, ModuleState state, long delay)
    {
        referenceModule=module;
        referenceState=state;
        delayTime=delay;
    }

    public Task(HwModule module, ModuleState state, RunCondition startCondition)
    {
        referenceModule=module;
        referenceState=state;
        condition =startCondition;
    }

    public Task(HwModule module, ModuleState state, long delay, RunCondition startCondition)
    {
        referenceModule=module;
        referenceState=state;
        condition =startCondition;
        delayTime=delay;
    }
}
