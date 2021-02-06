// Copyright 2020 Rensselaer Polytechnic Institute
//                Wason Technology, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using com.robotraconteur.geometry;
using com.robotraconteur.identifier;
using com.robotraconteur.robotics.robot;
using com.robotraconteur.robotics.tool;
using Mono.Options;
using Mono.Unix;
using RobotRaconteur;
using RobotRaconteur.Companion.InfoParser;
using RobotRaconteur.Companion.Util;

namespace SawyerRobotRaconteurDriver
{
    class Program
    {
        static int Main(string[] args)
        {

            bool shouldShowHelp = false;
            string robot_info_file = null;            
            bool wait_signal = false;
            bool electric_gripper = false;
            bool vacuum_gripper = false;
            string gripper_info_file = null;

            var options = new OptionSet {
                { "robot-info-file=", "the robot info YAML file", n => robot_info_file = n },
                { "h|help", "show this message and exit", h => shouldShowHelp = h != null },
                { "electric-gripper", "rethink electric gripper is attached", n=>electric_gripper = n!=null },
                { "vacuum-gripper", "rethink vacuum gripper is attached", n=>vacuum_gripper = n!=null },
                { "gripper-info-file=", "gripper info file", n=>gripper_info_file = n },
                {"wait-signal", "wait for POSIX sigint or sigkill to exit", n=> wait_signal = n!=null}
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

            if (vacuum_gripper && electric_gripper)
            {
                throw new ArgumentException("--vacuum-gripper and --electric-gripper are mutually exclusive");
            }

            Tuple<RobotInfo, LocalIdentifierLocks> robot_info = null;
            Tuple<ToolInfo, LocalIdentifierLocks> tool_info = null;
            SawyerRobot robot = null;
            ISawyerGripper gripper = null;
            

            try
            { 

                robot_info = RobotInfoParser.LoadRobotInfoYamlWithIdentifierLocks(robot_info_file);            

                ros_csharp_interop.ros_csharp_interop.init_ros(args, "sawyer_robotraconteur_driver", false);

                if (electric_gripper || vacuum_gripper)
                {
                    tool_info = ToolInfoParser.LoadToolInfoYamlWithIdentifierLocks(gripper_info_file);
                    tool_info.Item1.device_info.parent_device = robot_info.Item1.device_info.device;
                    tool_info.Item1.device_info.device_origin_pose = new NamedPose
                    {
                        parent_frame = new Identifier { name = "right_hand", uuid = new com.robotraconteur.uuid.UUID
                        {
                            uuid_bytes = new byte[16]
                        }
                        },
                        pose = new Pose { orientation = new Quaternion { w = 1 } }
                    };
                }

                robot = new SawyerRobot(robot_info.Item1, "");
                if (electric_gripper)
                {
                    gripper = new SawyerElectricGripper(tool_info.Item1, "right_gripper", "");
                    gripper._start_tool();
                }
                else if (vacuum_gripper)
                {
                    gripper = new SawyerVacuumGripper(tool_info.Item1, "right_vacuum_gripper", "");
                    gripper._start_tool();
                }

                robot._start_robot();
                using (var node_setup = new ServerNodeSetup("sawyer_robot", 58653, args))
                {
                    RobotRaconteurNode.s.RegisterService("robot", "com.robotraconteur.robotics.robot", robot);
                    if (gripper != null)
                    {
                        RobotRaconteurNode.s.RegisterService("gripper", "com.robotraconteur.robotics.tool", gripper);
                    }

                    if (!wait_signal)
                    {
                        Console.WriteLine("Press enter to exit");
                        Console.ReadKey();
                    }
                    else
                    {
                        UnixSignal[] signals = new UnixSignal[]{
                            new UnixSignal (Mono.Unix.Native.Signum.SIGINT),
                            new UnixSignal (Mono.Unix.Native.Signum.SIGTERM),
                        };

                        Console.WriteLine("Press Ctrl-C to exit");
                        // block until a SIGINT or SIGTERM signal is generated.
                        int which = UnixSignal.WaitAny(signals, -1);

                        Console.WriteLine("Got a {0} signal, exiting", signals[which].Signum);
                    }
                }
                
            }
            finally
            {
                robot_info?.Item2?.Dispose();
                tool_info?.Item2?.Dispose();
                robot?.Dispose();
                gripper?.Dispose();
            }

            return 0;

        }
    }
}
