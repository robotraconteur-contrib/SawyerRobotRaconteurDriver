using System;
using System.Collections.Generic;
using Mono.Options;
using RobotRaconteurWeb;

namespace SawyerRobotRaconteurDriver
{
    class Program
    {
        static int Main(string[] args)
        {

            bool shouldShowHelp = false;
            string robot_info_file = null;

            var options = new OptionSet {
                { "robot-info-file=", n => robot_info_file = n },
                { "h|help", "show this message and exit", h => shouldShowHelp = h != null }
            };
            
            List<string> extra;
            try
            {
                // parse the command line
                extra = options.Parse(args);
            }
            catch (OptionException e)
            {
                // output some error message
                Console.Write("SawyerRobotRaconteurDriver: ");
                Console.WriteLine(e.Message);
                Console.WriteLine("Try `SawyerRobotRaconteurDriver --help' for more information.");
                return 1;
            }

            if (shouldShowHelp)
            {
                Console.WriteLine("Usage: SawyerRobotRaconteurDriver [Options+]");
                Console.WriteLine();
                Console.WriteLine("Options:");
                options.WriteOptionDescriptions(Console.Out);
                return 0;
            }

            if (robot_info_file == null)
            {
                Console.WriteLine("error: robot-info-file must be specified");
                return 1;
            }


            var robot_info = RobotInfoParser.LoadRobotInfoYamlWithIdentifierLocks(robot_info_file);
            using (robot_info.Item2)
            {

                ros_csharp_interop.ros_csharp_interop.init_ros(args, "sawyer_robotraconteur_driver", false);


                using (var robot = new SawyerRobot(robot_info.Item1, ""))
                {
                    robot._start_robot();
                    using (var node_setup = new ServerNodeSetup("sawyer_robot", 58653))
                    {


                        RobotRaconteurNode.s.RegisterService("sawyer", "com.robotraconteur.robotics.robot", robot);

                        Console.WriteLine("Press enter to exit");
                        Console.ReadKey();

                        RobotRaconteurNode.s.Shutdown();
                    }
                }
            }

            return 0;

        }
    }
}
