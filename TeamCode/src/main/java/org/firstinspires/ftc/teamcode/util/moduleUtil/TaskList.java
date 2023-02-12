package org.firstinspires.ftc.teamcode.util.moduleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TaskList
{
    private List<Task> tasks=new ArrayList<>();
    boolean synchronous=false;

    public TaskList(Task... tasks)
    {
        this.tasks=Arrays.asList(tasks);
    }
    public TaskList(boolean synchronous, Task... tasks)
    {
        this.tasks=Arrays.asList(tasks);
        this.synchronous=synchronous;
    }

    public List<Task> getTasks()
    {
        return tasks;
    }
}
