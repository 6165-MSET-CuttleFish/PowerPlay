package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

public class ModuleController
{
    List<Module> modules;

    public ModuleController()
    {
        modules=new ArrayList<Module>();
    }

    public void add(Module m)
    {
        modules.add(m);
    }

    public void remove(Module m)
    {
        modules.remove(m);
    }

}
