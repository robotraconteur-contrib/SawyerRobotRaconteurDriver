using com.robotraconteur.robotics.tool;
using RobotRaconteur;
using RobotRaconteur.Companion.Robot;
using RobotRaconteur.Companion.Util;
using ros_csharp_interop;
using ros_csharp_interop.rosmsg.gen.intera_core_msgs;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.Threading;

namespace SawyerRobotRaconteurDriver
{
    public class SawyerElectricGripper : AbstractTool, ISawyerGripper
    {
        protected ROSNode _ros_node;
        protected string _ros_ns_prefix;
        protected string _ros_tool_name;

        protected Subscriber<IODeviceStatus> _gripper_state_sub;
        protected Publisher<IOComponentCommand> _gripper_command_pub;

        protected IODeviceStatus _gripper_current_state;
        protected Dictionary<string, string> _gripper_current_signals;
        protected long _last_gripper_state;
        protected double _last_command;
        protected string _tool_name;

        const double MAX_POSITION = 0.041667;
        const double MIN_POSITION = 0.0;
        const double MAX_VELOCITY = 3.0;
        const double MIN_VELOCITY = 0.15;
        const double POSITION_MARGIN = MAX_POSITION * 0.05;
        
        public SawyerElectricGripper(ToolInfo tool_info, string ros_tool_name, string ros_ns_prefix = "") : base(tool_info)
        {
            this._ros_ns_prefix = "";
            this._ros_tool_name = ros_tool_name;
        }


        public override void _start_tool()
        {
            _ros_node = new ROSNode();
            _gripper_state_sub = _ros_node.subscribe<IODeviceStatus>(_ros_ns_prefix + "io/end_effector/" + _ros_tool_name + "/state", 1, _gripper_state_cb); ;
            _gripper_command_pub = _ros_node.advertise<IOComponentCommand>(_ros_ns_prefix + "io/end_effector/" + _ros_tool_name + "/command", 1, false);

            base._start_tool();
        }

        private void _gripper_state_cb(IODeviceStatus obj)
        {
            lock(this)
            {                
                _gripper_current_state = obj;
                _last_gripper_state = _stopwatch.ElapsedMilliseconds;
                if (_gripper_current_signals == null)
                {
                    _gripper_current_signals = new Dictionary<string, string>();
                }
                else
                {
                    _gripper_current_signals.Clear();
                }

                foreach (var v in obj.signals)
                {
                    _gripper_current_signals[v.name] = v.data.Trim('[', ']');
                }
            }
        }

        private string _get_signal_or_default(string name, string default_)
        {
            if(!_gripper_current_signals.TryGetValue(name, out var v1))
            {
                return default_;
            }

            return v1;            
        }

        protected override void _fill_state(long now, out ToolState rr_tool_state)
        {
            lock (this)
            {
                var o = new ToolState();
                o.ts = DateTimeUtil.TimeSpec3Now(RobotRaconteurNode.s);
                o.seqno = _state_seqno;
                o.command = _last_command;

                if (_gripper_current_state == null || _gripper_current_signals == null 
                    // TODO: Gripper isn't publishing unless state changes?
                    //|| (now - _last_gripper_state > 2000)
                )
                {
                    o.tool_state_flags = (uint)ToolStateFlags.communication_failure;
                    o.sensor = new double[0];
                }
                else
                {
                    o.position = double.Parse(_get_signal_or_default("position_m","0"));
                    _position = o.position;
                    o.sensor = new double[] { double.Parse(_get_signal_or_default("force_response_m", "0")) };

                    bool has_error = bool.Parse(_get_signal_or_default("has_error", "true"));
                    bool is_calibrated = bool.Parse(_get_signal_or_default("is_calibrated", "false"));
                    bool is_moving = bool.Parse(_get_signal_or_default("is_moving", "false"));                    
                    bool ready = is_calibrated && !has_error && !is_moving;
                    bool is_gripping = bool.Parse(_get_signal_or_default("is_gripping", "false"));

                    uint f = 0;
                    if (has_error)
                    {
                        f |= (uint)ToolStateFlags.error;
                    }
                    if (!has_error)
                    {
                        f |= (uint)ToolStateFlags.enabled;
                    }
                    if (ready)
                    {
                        f |= (uint)ToolStateFlags.ready;
                    }
                    if (is_calibrated)
                    {
                        f |= (uint)ToolStateFlags.homed;
                    }
                    else
                    {
                        f |= (uint)ToolStateFlags.requires_homing;
                    }
                    if (is_gripping)
                    {
                        f |= (uint)ToolStateFlags.gripping;
                    }
                    if (is_moving)
                    {
                        f |= (uint)ToolStateFlags.actuating;
                    }
                    if (_position < POSITION_MARGIN)
                    {
                        f |= (uint)ToolStateFlags.closed;
                    }
                    else if (_position > MAX_POSITION - POSITION_MARGIN)
                    {
                        f |= (uint)ToolStateFlags.opened;
                    }
                    else
                    {
                        f |= (uint)ToolStateFlags.between;
                    }
                    o.tool_state_flags = f;
                }

                rr_tool_state = o;
                
            }
        }

        private TimeSpan _last_time;
        private ros_csharp_interop.rosmsg.ROSTime _now_ros()
        {
            var o = new ros_csharp_interop.rosmsg.ROSTime();
            TimeSpan t = DateTime.UtcNow.ToUniversalTime() - (new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc));

            if (t <= _last_time)
            {
                t = _last_time + TimeSpan.FromMilliseconds(1);
            }
            _last_time = t;

            o.secs = (uint)Math.Round(t.TotalSeconds);
            o.nsecs = (uint)Math.IEEERemainder(t.TotalMilliseconds * 1e6, 1e9);
            return o;
        }

        public override void home()
        {
            ros_csharp_interop.rosmsg.ROSTime t;
            lock (this)
            {
                t = _now_ros();
            }
            var cmd1 = new IOComponentCommand();
            cmd1.time = t;
            cmd1.op = "set";
            cmd1.args = "{\"signals\": {\"calibrate\": {\"data\": [true], \"format\": {\"type\": \"bool\" }}}}";
            _gripper_command_pub.publish(cmd1);

            Thread.Sleep(2500);
        }

        public override void close()
        {
            setf_command(MIN_POSITION);
        }

        public override void open()
        {
            setf_command(MAX_POSITION);
        }

        public override void setf_command(double command)
        {
            if (command < MIN_POSITION || command > MAX_POSITION)
            {
                throw new ArgumentException($"Command must be between {MIN_POSITION} and {MAX_POSITION}");
            }

            ros_csharp_interop.rosmsg.ROSTime t;
            lock (this)
            {
                t = _now_ros();
            }

            var cmd1 = new IOComponentCommand();
            cmd1.time = t;
            cmd1.op = "set";
            cmd1.args = "{\"signals\": {\"position_m\": {\"data\": [" + command.ToString() + "], \"format\": {\"type\": \"float\"}}}}";
  
            _gripper_command_pub.publish(cmd1);

            lock(this)
            {
                _command = command;
            }
        }

        public override void Dispose()
        {
            base.Dispose();
            _gripper_state_sub?.Dispose();

        }

        
    }
}
