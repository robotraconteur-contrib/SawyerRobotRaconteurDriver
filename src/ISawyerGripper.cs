using com.robotraconteur.robotics.tool;
using System;
using System.Collections.Generic;
using System.Text;

namespace SawyerRobotRaconteurDriver
{
    interface ISawyerGripper : Tool, IDisposable
    {
        void _start_tool();
    }
}
