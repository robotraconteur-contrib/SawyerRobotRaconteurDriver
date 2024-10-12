// Automatically generated, do not edit!

using System;
using System.IO;

namespace ros_csharp_interop.rosmsg.gen
{
    namespace std_msgs
    {
        [ROSMsgInfo("std_msgs/Header", "2176decaecbce78abc3b96ef049fabed", "# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n")]
        public class Header : ROSMsg
        {
            public string _type => "std_msgs/Header";
            public string _md5sum => "2176decaecbce78abc3b96ef049fabed";
            public string _full_text => "# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n";
            public uint seq = default;
            public ROSTime stamp = default;
            public string frame_id = default;
            public static Header ROSRead(BinaryReader reader)
            {
                var o = new Header();
                o.seq = rosmsg_builtin_util.read_uint(reader);
                o.stamp = rosmsg_builtin_util.read_ROSTime(reader);
                o.frame_id = rosmsg_builtin_util.read_string(reader);
                return o;
            }
            public static Header[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new Header[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, Header msg)
            {
                rosmsg_builtin_util.write_uint(writer, msg.seq);
                rosmsg_builtin_util.write_ROSTime(writer, msg.stamp);
                rosmsg_builtin_util.write_string(writer, msg.frame_id);
            }
            public static void ROSWriteArray(BinaryWriter writer, Header[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace sensor_msgs
    {
        [ROSMsgInfo("sensor_msgs/JointState", "3066dcd76a6cfaef579bd0f34173e9fd", "# This is a message that holds data to describe the state of a set of torque controlled joints. \n#\n# The state of each joint (revolute or prismatic) is defined by:\n#  * the position of the joint (rad or m),\n#  * the velocity of the joint (rad/s or m/s) and \n#  * the effort that is applied in the joint (Nm or N).\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state. \n# The goal is to make each of the fields optional. When e.g. your joints have no\n# effort associated with them, you can leave the effort array empty. \n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\n\nHeader header\n\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n")]
        public class JointState : ROSMsg
        {
            public string _type => "sensor_msgs/JointState";
            public string _md5sum => "3066dcd76a6cfaef579bd0f34173e9fd";
            public string _full_text => "# This is a message that holds data to describe the state of a set of torque controlled joints. \n#\n# The state of each joint (revolute or prismatic) is defined by:\n#  * the position of the joint (rad or m),\n#  * the velocity of the joint (rad/s or m/s) and \n#  * the effort that is applied in the joint (Nm or N).\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state. \n# The goal is to make each of the fields optional. When e.g. your joints have no\n# effort associated with them, you can leave the effort array empty. \n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\n\nHeader header\n\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n";
            public std_msgs.Header header = default;
            public string[] name = new string[0];
            public double[] position = new double[0];
            public double[] velocity = new double[0];
            public double[] effort = new double[0];
            public static JointState ROSRead(BinaryReader reader)
            {
                var o = new JointState();
                o.header = std_msgs.Header.ROSRead(reader);
                o.name = rosmsg_builtin_util.read_string_array(reader, -1);
                o.position = rosmsg_builtin_util.read_double_array(reader, -1);
                o.velocity = rosmsg_builtin_util.read_double_array(reader, -1);
                o.effort = rosmsg_builtin_util.read_double_array(reader, -1);
                return o;
            }
            public static JointState[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new JointState[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, JointState msg)
            {
                std_msgs.Header.ROSWrite(writer, msg.header);
                rosmsg_builtin_util.write_string_array(writer, msg.name, -1);
                rosmsg_builtin_util.write_double_array(writer, msg.position, -1);
                rosmsg_builtin_util.write_double_array(writer, msg.velocity, -1);
                rosmsg_builtin_util.write_double_array(writer, msg.effort, -1);
            }
            public static void ROSWriteArray(BinaryWriter writer, JointState[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/RobotAssemblyState", "df79ce5f75a6b23626e9fcdf1cc4d298", "bool homed               # true if all joints are homed\nbool ready               # true if enabled and ready to operate, e.g., not homing\nbool enabled             # true if enabled\nbool stopped             # true if stopped -- e-stop asserted\nbool error               # true if a component of the assembly has an error\nbool lowVoltage          # true when the robot is in low voltage mode\n\n# The following are specific to the robot top-level assembly:\nuint8  estop_button      # One of the following:\n  uint8   ESTOP_BUTTON_UNPRESSED = 0   # Robot is not stopped and button is not pressed\n  uint8   ESTOP_BUTTON_PRESSED   = 1\n  uint8   ESTOP_BUTTON_UNKNOWN   = 2   # STATE_UNKNOWN when estop was asserted by a non-user source\n  uint8   ESTOP_BUTTON_RELEASED  = 3   # Was pressed, is now known to be released, but robot is still stopped.\n#\nuint8  estop_source      # If stopped is true, the source of the e-stop.  One of the following:\n  uint8  ESTOP_SOURCE_NONE      = 0   # e-stop is not asserted\n  uint8  ESTOP_SOURCE_USER      = 1   # e-stop source is user input (the red button)\n  uint8  ESTOP_SOURCE_UNKNOWN   = 2   # e-stop source is unknown\n  uint8  ESTOP_SOURCE_FAULT     = 3   # MotorController asserted e-stop in response to a joint fault\n  uint8  ESTOP_SOURCE_ENGINE    = 4   # MotorController asserted e-stop in response to engine request\n")]
        public class RobotAssemblyState : ROSMsg
        {
            public string _type => "intera_core_msgs/RobotAssemblyState";
            public string _md5sum => "df79ce5f75a6b23626e9fcdf1cc4d298";
            public string _full_text => "bool homed               # true if all joints are homed\nbool ready               # true if enabled and ready to operate, e.g., not homing\nbool enabled             # true if enabled\nbool stopped             # true if stopped -- e-stop asserted\nbool error               # true if a component of the assembly has an error\nbool lowVoltage          # true when the robot is in low voltage mode\n\n# The following are specific to the robot top-level assembly:\nuint8  estop_button      # One of the following:\n  uint8   ESTOP_BUTTON_UNPRESSED = 0   # Robot is not stopped and button is not pressed\n  uint8   ESTOP_BUTTON_PRESSED   = 1\n  uint8   ESTOP_BUTTON_UNKNOWN   = 2   # STATE_UNKNOWN when estop was asserted by a non-user source\n  uint8   ESTOP_BUTTON_RELEASED  = 3   # Was pressed, is now known to be released, but robot is still stopped.\n#\nuint8  estop_source      # If stopped is true, the source of the e-stop.  One of the following:\n  uint8  ESTOP_SOURCE_NONE      = 0   # e-stop is not asserted\n  uint8  ESTOP_SOURCE_USER      = 1   # e-stop source is user input (the red button)\n  uint8  ESTOP_SOURCE_UNKNOWN   = 2   # e-stop source is unknown\n  uint8  ESTOP_SOURCE_FAULT     = 3   # MotorController asserted e-stop in response to a joint fault\n  uint8  ESTOP_SOURCE_ENGINE    = 4   # MotorController asserted e-stop in response to engine request\n";
            public bool homed = default;
            public bool ready = default;
            public bool enabled = default;
            public bool stopped = default;
            public bool error = default;
            public bool lowVoltage = default;
            public byte estop_button = default;
            public byte estop_source = default;
            public static RobotAssemblyState ROSRead(BinaryReader reader)
            {
                var o = new RobotAssemblyState();
                o.homed = rosmsg_builtin_util.read_bool(reader);
                o.ready = rosmsg_builtin_util.read_bool(reader);
                o.enabled = rosmsg_builtin_util.read_bool(reader);
                o.stopped = rosmsg_builtin_util.read_bool(reader);
                o.error = rosmsg_builtin_util.read_bool(reader);
                o.lowVoltage = rosmsg_builtin_util.read_bool(reader);
                o.estop_button = rosmsg_builtin_util.read_byte(reader);
                o.estop_source = rosmsg_builtin_util.read_byte(reader);
                return o;
            }
            public static RobotAssemblyState[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new RobotAssemblyState[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, RobotAssemblyState msg)
            {
                rosmsg_builtin_util.write_bool(writer, msg.homed);
                rosmsg_builtin_util.write_bool(writer, msg.ready);
                rosmsg_builtin_util.write_bool(writer, msg.enabled);
                rosmsg_builtin_util.write_bool(writer, msg.stopped);
                rosmsg_builtin_util.write_bool(writer, msg.error);
                rosmsg_builtin_util.write_bool(writer, msg.lowVoltage);
                rosmsg_builtin_util.write_byte(writer, msg.estop_button);
                rosmsg_builtin_util.write_byte(writer, msg.estop_source);
            }
            public static void ROSWriteArray(BinaryWriter writer, RobotAssemblyState[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/JointCommand", "c8c85922d297da6209a089a906207e5d", "Header header\n\nint32 mode             # Mode in which to command arm\n\nstring[]  names        # Joint names order for command\n\n# Fields of commands indexed according to the Joint names vector.\n# Command fields required for a desired mode are listed in the comments\nfloat64[] position     # (radians)       Required for POSITION_MODE and TRAJECTORY_MODE\nfloat64[] velocity     # (radians/sec)   Required for VELOCITY_MODE and TRAJECTORY_MODE\nfloat64[] acceleration # (radians/sec^2) Required for                   TRAJECTORY_MODE\nfloat64[] effort       # (newton-meters) Required for TORQUE_MODE\n\n# Modes available to command arm\nint32 POSITION_MODE=1\nint32 VELOCITY_MODE=2\nint32 TORQUE_MODE=3\nint32 TRAJECTORY_MODE=4\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n")]
        public class JointCommand : ROSMsg
        {
            public string _type => "intera_core_msgs/JointCommand";
            public string _md5sum => "c8c85922d297da6209a089a906207e5d";
            public string _full_text => "Header header\n\nint32 mode             # Mode in which to command arm\n\nstring[]  names        # Joint names order for command\n\n# Fields of commands indexed according to the Joint names vector.\n# Command fields required for a desired mode are listed in the comments\nfloat64[] position     # (radians)       Required for POSITION_MODE and TRAJECTORY_MODE\nfloat64[] velocity     # (radians/sec)   Required for VELOCITY_MODE and TRAJECTORY_MODE\nfloat64[] acceleration # (radians/sec^2) Required for                   TRAJECTORY_MODE\nfloat64[] effort       # (newton-meters) Required for TORQUE_MODE\n\n# Modes available to command arm\nint32 POSITION_MODE=1\nint32 VELOCITY_MODE=2\nint32 TORQUE_MODE=3\nint32 TRAJECTORY_MODE=4\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n";
            public std_msgs.Header header = default;
            public int mode = default;
            public string[] names = new string[0];
            public double[] position = new double[0];
            public double[] velocity = new double[0];
            public double[] acceleration = new double[0];
            public double[] effort = new double[0];
            public static JointCommand ROSRead(BinaryReader reader)
            {
                var o = new JointCommand();
                o.header = std_msgs.Header.ROSRead(reader);
                o.mode = rosmsg_builtin_util.read_int(reader);
                o.names = rosmsg_builtin_util.read_string_array(reader, -1);
                o.position = rosmsg_builtin_util.read_double_array(reader, -1);
                o.velocity = rosmsg_builtin_util.read_double_array(reader, -1);
                o.acceleration = rosmsg_builtin_util.read_double_array(reader, -1);
                o.effort = rosmsg_builtin_util.read_double_array(reader, -1);
                return o;
            }
            public static JointCommand[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new JointCommand[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, JointCommand msg)
            {
                std_msgs.Header.ROSWrite(writer, msg.header);
                rosmsg_builtin_util.write_int(writer, msg.mode);
                rosmsg_builtin_util.write_string_array(writer, msg.names, -1);
                rosmsg_builtin_util.write_double_array(writer, msg.position, -1);
                rosmsg_builtin_util.write_double_array(writer, msg.velocity, -1);
                rosmsg_builtin_util.write_double_array(writer, msg.acceleration, -1);
                rosmsg_builtin_util.write_double_array(writer, msg.effort, -1);
            }
            public static void ROSWriteArray(BinaryWriter writer, JointCommand[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace std_msgs
    {
        [ROSMsgInfo("std_msgs/Empty", "d41d8cd98f00b204e9800998ecf8427e", "")]
        public class Empty : ROSMsg
        {
            public string _type => "std_msgs/Empty";
            public string _md5sum => "d41d8cd98f00b204e9800998ecf8427e";
            public string _full_text => "";
            public static Empty ROSRead(BinaryReader reader)
            {
                var o = new Empty();
                return o;
            }
            public static Empty[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new Empty[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, Empty msg)
            {
            }
            public static void ROSWriteArray(BinaryWriter writer, Empty[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace std_msgs
    {
        [ROSMsgInfo("std_msgs/Bool", "8b94c1b53db61fb6aed406028ad6332a", "bool data")]
        public class Bool : ROSMsg
        {
            public string _type => "std_msgs/Bool";
            public string _md5sum => "8b94c1b53db61fb6aed406028ad6332a";
            public string _full_text => "bool data";
            public bool data = default;
            public static Bool ROSRead(BinaryReader reader)
            {
                var o = new Bool();
                o.data = rosmsg_builtin_util.read_bool(reader);
                return o;
            }
            public static Bool[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new Bool[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, Bool msg)
            {
                rosmsg_builtin_util.write_bool(writer, msg.data);
            }
            public static void ROSWriteArray(BinaryWriter writer, Bool[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/EndpointState", "ec04f0f9bc02b3e335181c07d68c2c98", "Header               header\ngeometry_msgs/Pose   pose\ngeometry_msgs/Twist  twist\ngeometry_msgs/Wrench wrench\nbool                 valid\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Wrench\n# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n")]
        public class EndpointState : ROSMsg
        {
            public string _type => "intera_core_msgs/EndpointState";
            public string _md5sum => "ec04f0f9bc02b3e335181c07d68c2c98";
            public string _full_text => "Header               header\ngeometry_msgs/Pose   pose\ngeometry_msgs/Twist  twist\ngeometry_msgs/Wrench wrench\nbool                 valid\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Wrench\n# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n";
            public std_msgs.Header header = default;
            public geometry_msgs.Pose pose = default;
            public geometry_msgs.Twist twist = default;
            public geometry_msgs.Wrench wrench = default;
            public bool valid = default;
            public static EndpointState ROSRead(BinaryReader reader)
            {
                var o = new EndpointState();
                o.header = std_msgs.Header.ROSRead(reader);
                o.pose = geometry_msgs.Pose.ROSRead(reader);
                o.twist = geometry_msgs.Twist.ROSRead(reader);
                o.wrench = geometry_msgs.Wrench.ROSRead(reader);
                o.valid = rosmsg_builtin_util.read_bool(reader);
                return o;
            }
            public static EndpointState[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new EndpointState[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, EndpointState msg)
            {
                std_msgs.Header.ROSWrite(writer, msg.header);
                geometry_msgs.Pose.ROSWrite(writer, msg.pose);
                geometry_msgs.Twist.ROSWrite(writer, msg.twist);
                geometry_msgs.Wrench.ROSWrite(writer, msg.wrench);
                rosmsg_builtin_util.write_bool(writer, msg.valid);
            }
            public static void ROSWriteArray(BinaryWriter writer, EndpointState[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace geometry_msgs
    {
        [ROSMsgInfo("geometry_msgs/Pose", "e45d45a5a1ce597b249e23fb30fc871f", "# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n")]
        public class Pose : ROSMsg
        {
            public string _type => "geometry_msgs/Pose";
            public string _md5sum => "e45d45a5a1ce597b249e23fb30fc871f";
            public string _full_text => "# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n";
            public geometry_msgs.Point position = default;
            public geometry_msgs.Quaternion orientation = default;
            public static Pose ROSRead(BinaryReader reader)
            {
                var o = new Pose();
                o.position = geometry_msgs.Point.ROSRead(reader);
                o.orientation = geometry_msgs.Quaternion.ROSRead(reader);
                return o;
            }
            public static Pose[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new Pose[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, Pose msg)
            {
                geometry_msgs.Point.ROSWrite(writer, msg.position);
                geometry_msgs.Quaternion.ROSWrite(writer, msg.orientation);
            }
            public static void ROSWriteArray(BinaryWriter writer, Pose[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace geometry_msgs
    {
        [ROSMsgInfo("geometry_msgs/Twist", "9f195f881246fdfa2798d1d3eebca84a", "# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z")]
        public class Twist : ROSMsg
        {
            public string _type => "geometry_msgs/Twist";
            public string _md5sum => "9f195f881246fdfa2798d1d3eebca84a";
            public string _full_text => "# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z";
            public geometry_msgs.Vector3 linear = default;
            public geometry_msgs.Vector3 angular = default;
            public static Twist ROSRead(BinaryReader reader)
            {
                var o = new Twist();
                o.linear = geometry_msgs.Vector3.ROSRead(reader);
                o.angular = geometry_msgs.Vector3.ROSRead(reader);
                return o;
            }
            public static Twist[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new Twist[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, Twist msg)
            {
                geometry_msgs.Vector3.ROSWrite(writer, msg.linear);
                geometry_msgs.Vector3.ROSWrite(writer, msg.angular);
            }
            public static void ROSWriteArray(BinaryWriter writer, Twist[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace geometry_msgs
    {
        [ROSMsgInfo("geometry_msgs/Wrench", "4f539cf138b23283b520fd271b567936", "# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z")]
        public class Wrench : ROSMsg
        {
            public string _type => "geometry_msgs/Wrench";
            public string _md5sum => "4f539cf138b23283b520fd271b567936";
            public string _full_text => "# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z";
            public geometry_msgs.Vector3 force = default;
            public geometry_msgs.Vector3 torque = default;
            public static Wrench ROSRead(BinaryReader reader)
            {
                var o = new Wrench();
                o.force = geometry_msgs.Vector3.ROSRead(reader);
                o.torque = geometry_msgs.Vector3.ROSRead(reader);
                return o;
            }
            public static Wrench[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new Wrench[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, Wrench msg)
            {
                geometry_msgs.Vector3.ROSWrite(writer, msg.force);
                geometry_msgs.Vector3.ROSWrite(writer, msg.torque);
            }
            public static void ROSWriteArray(BinaryWriter writer, Wrench[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace geometry_msgs
    {
        [ROSMsgInfo("geometry_msgs/Point", "4a842b65f413084dc2b10fb484ea7f17", "# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n")]
        public class Point : ROSMsg
        {
            public string _type => "geometry_msgs/Point";
            public string _md5sum => "4a842b65f413084dc2b10fb484ea7f17";
            public string _full_text => "# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n";
            public double x = default;
            public double y = default;
            public double z = default;
            public static Point ROSRead(BinaryReader reader)
            {
                var o = new Point();
                o.x = rosmsg_builtin_util.read_double(reader);
                o.y = rosmsg_builtin_util.read_double(reader);
                o.z = rosmsg_builtin_util.read_double(reader);
                return o;
            }
            public static Point[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new Point[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, Point msg)
            {
                rosmsg_builtin_util.write_double(writer, msg.x);
                rosmsg_builtin_util.write_double(writer, msg.y);
                rosmsg_builtin_util.write_double(writer, msg.z);
            }
            public static void ROSWriteArray(BinaryWriter writer, Point[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace geometry_msgs
    {
        [ROSMsgInfo("geometry_msgs/Quaternion", "a779879fadf0160734f906b8c19c7004", "# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n")]
        public class Quaternion : ROSMsg
        {
            public string _type => "geometry_msgs/Quaternion";
            public string _md5sum => "a779879fadf0160734f906b8c19c7004";
            public string _full_text => "# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n";
            public double x = default;
            public double y = default;
            public double z = default;
            public double w = default;
            public static Quaternion ROSRead(BinaryReader reader)
            {
                var o = new Quaternion();
                o.x = rosmsg_builtin_util.read_double(reader);
                o.y = rosmsg_builtin_util.read_double(reader);
                o.z = rosmsg_builtin_util.read_double(reader);
                o.w = rosmsg_builtin_util.read_double(reader);
                return o;
            }
            public static Quaternion[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new Quaternion[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, Quaternion msg)
            {
                rosmsg_builtin_util.write_double(writer, msg.x);
                rosmsg_builtin_util.write_double(writer, msg.y);
                rosmsg_builtin_util.write_double(writer, msg.z);
                rosmsg_builtin_util.write_double(writer, msg.w);
            }
            public static void ROSWriteArray(BinaryWriter writer, Quaternion[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace geometry_msgs
    {
        [ROSMsgInfo("geometry_msgs/Vector3", "4a842b65f413084dc2b10fb484ea7f17", "# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z")]
        public class Vector3 : ROSMsg
        {
            public string _type => "geometry_msgs/Vector3";
            public string _md5sum => "4a842b65f413084dc2b10fb484ea7f17";
            public string _full_text => "# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z";
            public double x = default;
            public double y = default;
            public double z = default;
            public static Vector3 ROSRead(BinaryReader reader)
            {
                var o = new Vector3();
                o.x = rosmsg_builtin_util.read_double(reader);
                o.y = rosmsg_builtin_util.read_double(reader);
                o.z = rosmsg_builtin_util.read_double(reader);
                return o;
            }
            public static Vector3[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new Vector3[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, Vector3 msg)
            {
                rosmsg_builtin_util.write_double(writer, msg.x);
                rosmsg_builtin_util.write_double(writer, msg.y);
                rosmsg_builtin_util.write_double(writer, msg.z);
            }
            public static void ROSWriteArray(BinaryWriter writer, Vector3[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/HomingCommand", "ff29c86109f0f4cada5bbde945dd55c4", "# This message is used to command the homing state of all joints on the robot.\n# For each joint in the name array the corresponding value in the command\n# sets the homing mode for that joint.\n# MANUAL disengages the joint brake and puts the joint in a \"gumby\" mode.\n# AUTO disengages the joint brake and automatically homes the joint.\n# If a joint is already homed the command has no effect.\nstring[] name\nint32[]  command\n# Valid homing commands:\nint32 MANUAL=1\nint32 AUTO=2\n# Invalid, value is used internally:\nint32 NONE=0\n\n\n")]
        public class HomingCommand : ROSMsg
        {
            public string _type => "intera_core_msgs/HomingCommand";
            public string _md5sum => "ff29c86109f0f4cada5bbde945dd55c4";
            public string _full_text => "# This message is used to command the homing state of all joints on the robot.\n# For each joint in the name array the corresponding value in the command\n# sets the homing mode for that joint.\n# MANUAL disengages the joint brake and puts the joint in a \"gumby\" mode.\n# AUTO disengages the joint brake and automatically homes the joint.\n# If a joint is already homed the command has no effect.\nstring[] name\nint32[]  command\n# Valid homing commands:\nint32 MANUAL=1\nint32 AUTO=2\n# Invalid, value is used internally:\nint32 NONE=0\n\n\n";
            public string[] name = new string[0];
            public int[] command = new int[0];
            public static HomingCommand ROSRead(BinaryReader reader)
            {
                var o = new HomingCommand();
                o.name = rosmsg_builtin_util.read_string_array(reader, -1);
                o.command = rosmsg_builtin_util.read_int_array(reader, -1);
                return o;
            }
            public static HomingCommand[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new HomingCommand[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, HomingCommand msg)
            {
                rosmsg_builtin_util.write_string_array(writer, msg.name, -1);
                rosmsg_builtin_util.write_int_array(writer, msg.command, -1);
            }
            public static void ROSWriteArray(BinaryWriter writer, HomingCommand[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/DigitalIOState", "29d0be3859dae81a66b28f167ecec98c", "int8 state\nbool isInputOnly\n\nint8 OFF = 0\nint8 ON  = 1\nint8 PRESSED = 1\nint8 UNPRESSED = 0")]
        public class DigitalIOState : ROSMsg
        {
            public string _type => "intera_core_msgs/DigitalIOState";
            public string _md5sum => "29d0be3859dae81a66b28f167ecec98c";
            public string _full_text => "int8 state\nbool isInputOnly\n\nint8 OFF = 0\nint8 ON  = 1\nint8 PRESSED = 1\nint8 UNPRESSED = 0";
            public sbyte state = default;
            public bool isInputOnly = default;
            public static DigitalIOState ROSRead(BinaryReader reader)
            {
                var o = new DigitalIOState();
                o.state = rosmsg_builtin_util.read_sbyte(reader);
                o.isInputOnly = rosmsg_builtin_util.read_bool(reader);
                return o;
            }
            public static DigitalIOState[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new DigitalIOState[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, DigitalIOState msg)
            {
                rosmsg_builtin_util.write_sbyte(writer, msg.state);
                rosmsg_builtin_util.write_bool(writer, msg.isInputOnly);
            }
            public static void ROSWriteArray(BinaryWriter writer, DigitalIOState[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/DigitalIOStates", "d434210c6ec20947fec667d6c13d6062", "string[]         names\nDigitalIOState[] states\n================================================================================\nMSG: intera_core_msgs/DigitalIOState\nint8 state\nbool isInputOnly\n\nint8 OFF = 0\nint8 ON  = 1\nint8 PRESSED = 1\nint8 UNPRESSED = 0")]
        public class DigitalIOStates : ROSMsg
        {
            public string _type => "intera_core_msgs/DigitalIOStates";
            public string _md5sum => "d434210c6ec20947fec667d6c13d6062";
            public string _full_text => "string[]         names\nDigitalIOState[] states\n================================================================================\nMSG: intera_core_msgs/DigitalIOState\nint8 state\nbool isInputOnly\n\nint8 OFF = 0\nint8 ON  = 1\nint8 PRESSED = 1\nint8 UNPRESSED = 0";
            public string[] names = new string[0];
            public intera_core_msgs.DigitalIOState[] states = new intera_core_msgs.DigitalIOState[0];
            public static DigitalIOStates ROSRead(BinaryReader reader)
            {
                var o = new DigitalIOStates();
                o.names = rosmsg_builtin_util.read_string_array(reader, -1);
                o.states = intera_core_msgs.DigitalIOState.ROSReadArray(reader, -1);
                return o;
            }
            public static DigitalIOStates[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new DigitalIOStates[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, DigitalIOStates msg)
            {
                rosmsg_builtin_util.write_string_array(writer, msg.names, -1);
                intera_core_msgs.DigitalIOState.ROSWriteArray(writer, msg.states, -1);
            }
            public static void ROSWriteArray(BinaryWriter writer, DigitalIOStates[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/DigitalOutputCommand", "23f05028c1a699fb83e22401228c3a9e", "##the name of the output\nstring name  \n##the value to set output \nbool value   \n")]
        public class DigitalOutputCommand : ROSMsg
        {
            public string _type => "intera_core_msgs/DigitalOutputCommand";
            public string _md5sum => "23f05028c1a699fb83e22401228c3a9e";
            public string _full_text => "##the name of the output\nstring name  \n##the value to set output \nbool value   \n";
            public string name = default;
            public bool value = default;
            public static DigitalOutputCommand ROSRead(BinaryReader reader)
            {
                var o = new DigitalOutputCommand();
                o.name = rosmsg_builtin_util.read_string(reader);
                o.value = rosmsg_builtin_util.read_bool(reader);
                return o;
            }
            public static DigitalOutputCommand[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new DigitalOutputCommand[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, DigitalOutputCommand msg)
            {
                rosmsg_builtin_util.write_string(writer, msg.name);
                rosmsg_builtin_util.write_bool(writer, msg.value);
            }
            public static void ROSWriteArray(BinaryWriter writer, DigitalOutputCommand[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/AnalogIOState", "39af371963dc9e4447e91f430c720b33", "time timestamp\nfloat64 value\nbool isInputOnly\n")]
        public class AnalogIOState : ROSMsg
        {
            public string _type => "intera_core_msgs/AnalogIOState";
            public string _md5sum => "39af371963dc9e4447e91f430c720b33";
            public string _full_text => "time timestamp\nfloat64 value\nbool isInputOnly\n";
            public ROSTime timestamp = default;
            public double value = default;
            public bool isInputOnly = default;
            public static AnalogIOState ROSRead(BinaryReader reader)
            {
                var o = new AnalogIOState();
                o.timestamp = rosmsg_builtin_util.read_ROSTime(reader);
                o.value = rosmsg_builtin_util.read_double(reader);
                o.isInputOnly = rosmsg_builtin_util.read_bool(reader);
                return o;
            }
            public static AnalogIOState[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new AnalogIOState[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, AnalogIOState msg)
            {
                rosmsg_builtin_util.write_ROSTime(writer, msg.timestamp);
                rosmsg_builtin_util.write_double(writer, msg.value);
                rosmsg_builtin_util.write_bool(writer, msg.isInputOnly);
            }
            public static void ROSWriteArray(BinaryWriter writer, AnalogIOState[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/AnalogIOStates", "0a05ba3bbb53a8a3b164e34946d619f7", "string[]         names\nAnalogIOState[] states\n================================================================================\nMSG: intera_core_msgs/AnalogIOState\ntime timestamp\nfloat64 value\nbool isInputOnly\n")]
        public class AnalogIOStates : ROSMsg
        {
            public string _type => "intera_core_msgs/AnalogIOStates";
            public string _md5sum => "0a05ba3bbb53a8a3b164e34946d619f7";
            public string _full_text => "string[]         names\nAnalogIOState[] states\n================================================================================\nMSG: intera_core_msgs/AnalogIOState\ntime timestamp\nfloat64 value\nbool isInputOnly\n";
            public string[] names = new string[0];
            public intera_core_msgs.AnalogIOState[] states = new intera_core_msgs.AnalogIOState[0];
            public static AnalogIOStates ROSRead(BinaryReader reader)
            {
                var o = new AnalogIOStates();
                o.names = rosmsg_builtin_util.read_string_array(reader, -1);
                o.states = intera_core_msgs.AnalogIOState.ROSReadArray(reader, -1);
                return o;
            }
            public static AnalogIOStates[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new AnalogIOStates[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, AnalogIOStates msg)
            {
                rosmsg_builtin_util.write_string_array(writer, msg.names, -1);
                intera_core_msgs.AnalogIOState.ROSWriteArray(writer, msg.states, -1);
            }
            public static void ROSWriteArray(BinaryWriter writer, AnalogIOStates[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/AnalogOutputCommand", "a7b945129a083ca4095d48aa94841d85", "##the name of the output\nstring name  \n##the value to set output \nuint16 value   \n")]
        public class AnalogOutputCommand : ROSMsg
        {
            public string _type => "intera_core_msgs/AnalogOutputCommand";
            public string _md5sum => "a7b945129a083ca4095d48aa94841d85";
            public string _full_text => "##the name of the output\nstring name  \n##the value to set output \nuint16 value   \n";
            public string name = default;
            public ushort value = default;
            public static AnalogOutputCommand ROSRead(BinaryReader reader)
            {
                var o = new AnalogOutputCommand();
                o.name = rosmsg_builtin_util.read_string(reader);
                o.value = rosmsg_builtin_util.read_ushort(reader);
                return o;
            }
            public static AnalogOutputCommand[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new AnalogOutputCommand[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, AnalogOutputCommand msg)
            {
                rosmsg_builtin_util.write_string(writer, msg.name);
                rosmsg_builtin_util.write_ushort(writer, msg.value);
            }
            public static void ROSWriteArray(BinaryWriter writer, AnalogOutputCommand[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/IOComponentCommand", "ede95ba2953dc221dc82cac20f697530", "## IO Component Command\ntime time      # time the message was created, serves as a sequence number\nstring op      # operation to perform\nstring args    # JSON arguments\n")]
        public class IOComponentCommand : ROSMsg
        {
            public string _type => "intera_core_msgs/IOComponentCommand";
            public string _md5sum => "ede95ba2953dc221dc82cac20f697530";
            public string _full_text => "## IO Component Command\ntime time      # time the message was created, serves as a sequence number\nstring op      # operation to perform\nstring args    # JSON arguments\n";
            public ROSTime time = default;
            public string op = default;
            public string args = default;
            public static IOComponentCommand ROSRead(BinaryReader reader)
            {
                var o = new IOComponentCommand();
                o.time = rosmsg_builtin_util.read_ROSTime(reader);
                o.op = rosmsg_builtin_util.read_string(reader);
                o.args = rosmsg_builtin_util.read_string(reader);
                return o;
            }
            public static IOComponentCommand[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new IOComponentCommand[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, IOComponentCommand msg)
            {
                rosmsg_builtin_util.write_ROSTime(writer, msg.time);
                rosmsg_builtin_util.write_string(writer, msg.op);
                rosmsg_builtin_util.write_string(writer, msg.args);
            }
            public static void ROSWriteArray(BinaryWriter writer, IOComponentCommand[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/IOComponentConfiguration", "cb7717d13a521b51b5c0a02d493c42fd", "## IO Component configuration data\nstring name                           # component name\nstring config                         # component configuration JSON\n")]
        public class IOComponentConfiguration : ROSMsg
        {
            public string _type => "intera_core_msgs/IOComponentConfiguration";
            public string _md5sum => "cb7717d13a521b51b5c0a02d493c42fd";
            public string _full_text => "## IO Component configuration data\nstring name                           # component name\nstring config                         # component configuration JSON\n";
            public string name = default;
            public string config = default;
            public static IOComponentConfiguration ROSRead(BinaryReader reader)
            {
                var o = new IOComponentConfiguration();
                o.name = rosmsg_builtin_util.read_string(reader);
                o.config = rosmsg_builtin_util.read_string(reader);
                return o;
            }
            public static IOComponentConfiguration[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new IOComponentConfiguration[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, IOComponentConfiguration msg)
            {
                rosmsg_builtin_util.write_string(writer, msg.name);
                rosmsg_builtin_util.write_string(writer, msg.config);
            }
            public static void ROSWriteArray(BinaryWriter writer, IOComponentConfiguration[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/IODataStatus", "bb31283c6afc4ddea2f5f157264e5909", "## IO Data Status\nstring name       # IO Data Component name\nstring format     # data format:\n                  # A JSON object containing one or more of the following fields:\n                  # Required:\n                  #   \"type\" : \"<type>\"  JSON Type, one of:\n                  #                      \"bool\", \"int\", \"float\", \"string\", \"object\", \"array\"\n                  #\n                  # Optional:\n                  #   \"dimensions\" : [ N,...] if absent, [ 1 ] is assumed.\n                  #   \"role\"       : Signal: \"input\", \"output\"; Port:  \"sink\", \"source\"\n                  #   \"data_type\"  : qualifier for the JSON type, may be device-specific\n                  #                  int data_types:  \"uN\", \"sN\", \"NqM\"\n                  #   other device-specific qualifiers may be used and will be passed through.\nstring data       # JSON data value.  An Array is expected, for example \"[ 0 ]\"\nIOStatus status   # Data Component status\n\n================================================================================\nMSG: intera_core_msgs/IOStatus\n## IO status data\n#\nstring tag             # one of the values listed below\n#   down     Inoperative, not fully instantiated\n#   ready    OK, fully operational\n#   busy     OK, not ready to output data; input data value may be stale\n#   unready  OK, not operational; data is invalid\n#   error    Error, not operational\nstring DOWN      = down\nstring READY     = ready\nstring BUSY      = busy\nstring UNREADY   = unready\nstring ERROR     = error\n#\nstring id             # message id, for internationalization\n#\nstring detail         # optional additional status detail\n#\n")]
        public class IODataStatus : ROSMsg
        {
            public string _type => "intera_core_msgs/IODataStatus";
            public string _md5sum => "bb31283c6afc4ddea2f5f157264e5909";
            public string _full_text => "## IO Data Status\nstring name       # IO Data Component name\nstring format     # data format:\n                  # A JSON object containing one or more of the following fields:\n                  # Required:\n                  #   \"type\" : \"<type>\"  JSON Type, one of:\n                  #                      \"bool\", \"int\", \"float\", \"string\", \"object\", \"array\"\n                  #\n                  # Optional:\n                  #   \"dimensions\" : [ N,...] if absent, [ 1 ] is assumed.\n                  #   \"role\"       : Signal: \"input\", \"output\"; Port:  \"sink\", \"source\"\n                  #   \"data_type\"  : qualifier for the JSON type, may be device-specific\n                  #                  int data_types:  \"uN\", \"sN\", \"NqM\"\n                  #   other device-specific qualifiers may be used and will be passed through.\nstring data       # JSON data value.  An Array is expected, for example \"[ 0 ]\"\nIOStatus status   # Data Component status\n\n================================================================================\nMSG: intera_core_msgs/IOStatus\n## IO status data\n#\nstring tag             # one of the values listed below\n#   down     Inoperative, not fully instantiated\n#   ready    OK, fully operational\n#   busy     OK, not ready to output data; input data value may be stale\n#   unready  OK, not operational; data is invalid\n#   error    Error, not operational\nstring DOWN      = down\nstring READY     = ready\nstring BUSY      = busy\nstring UNREADY   = unready\nstring ERROR     = error\n#\nstring id             # message id, for internationalization\n#\nstring detail         # optional additional status detail\n#\n";
            public string name = default;
            public string format = default;
            public string data = default;
            public intera_core_msgs.IOStatus status = default;
            public static IODataStatus ROSRead(BinaryReader reader)
            {
                var o = new IODataStatus();
                o.name = rosmsg_builtin_util.read_string(reader);
                o.format = rosmsg_builtin_util.read_string(reader);
                o.data = rosmsg_builtin_util.read_string(reader);
                o.status = intera_core_msgs.IOStatus.ROSRead(reader);
                return o;
            }
            public static IODataStatus[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new IODataStatus[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, IODataStatus msg)
            {
                rosmsg_builtin_util.write_string(writer, msg.name);
                rosmsg_builtin_util.write_string(writer, msg.format);
                rosmsg_builtin_util.write_string(writer, msg.data);
                intera_core_msgs.IOStatus.ROSWrite(writer, msg.status);
            }
            public static void ROSWriteArray(BinaryWriter writer, IODataStatus[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/IONodeConfiguration", "66800b32dbb52df00e2454d58668ac33", "# IO Node Configuration\ntime time                           # configuration command timestamp\nIOComponentConfiguration node       # IO Node configuration\nIOComponentConfiguration[] devices  # configuration of IO Devices in this node\nIOComponentConfiguration[] plugins  # configuration of IO Device Plugins in this node\n\n================================================================================\nMSG: intera_core_msgs/IOComponentConfiguration\n## IO Component configuration data\nstring name                           # component name\nstring config                         # component configuration JSON\n")]
        public class IONodeConfiguration : ROSMsg
        {
            public string _type => "intera_core_msgs/IONodeConfiguration";
            public string _md5sum => "66800b32dbb52df00e2454d58668ac33";
            public string _full_text => "# IO Node Configuration\ntime time                           # configuration command timestamp\nIOComponentConfiguration node       # IO Node configuration\nIOComponentConfiguration[] devices  # configuration of IO Devices in this node\nIOComponentConfiguration[] plugins  # configuration of IO Device Plugins in this node\n\n================================================================================\nMSG: intera_core_msgs/IOComponentConfiguration\n## IO Component configuration data\nstring name                           # component name\nstring config                         # component configuration JSON\n";
            public ROSTime time = default;
            public intera_core_msgs.IOComponentConfiguration node = default;
            public intera_core_msgs.IOComponentConfiguration[] devices = new intera_core_msgs.IOComponentConfiguration[0];
            public intera_core_msgs.IOComponentConfiguration[] plugins = new intera_core_msgs.IOComponentConfiguration[0];
            public static IONodeConfiguration ROSRead(BinaryReader reader)
            {
                var o = new IONodeConfiguration();
                o.time = rosmsg_builtin_util.read_ROSTime(reader);
                o.node = intera_core_msgs.IOComponentConfiguration.ROSRead(reader);
                o.devices = intera_core_msgs.IOComponentConfiguration.ROSReadArray(reader, -1);
                o.plugins = intera_core_msgs.IOComponentConfiguration.ROSReadArray(reader, -1);
                return o;
            }
            public static IONodeConfiguration[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new IONodeConfiguration[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, IONodeConfiguration msg)
            {
                rosmsg_builtin_util.write_ROSTime(writer, msg.time);
                intera_core_msgs.IOComponentConfiguration.ROSWrite(writer, msg.node);
                intera_core_msgs.IOComponentConfiguration.ROSWriteArray(writer, msg.devices, -1);
                intera_core_msgs.IOComponentConfiguration.ROSWriteArray(writer, msg.plugins, -1);
            }
            public static void ROSWriteArray(BinaryWriter writer, IONodeConfiguration[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/IODeviceStatus", "0d0c97a2d700848e7ad46e09a77cf896", "## IO Device status\ntime time                  # time the message was created\nIOComponentStatus device   # device status data\nIODataStatus[]    ports    # Ports status\nIODataStatus[]    signals  # Signals status\ntime[]            commands # recent command timestamps, for syncing\nstring[]          responses # recent command responses\n\n\n================================================================================\nMSG: intera_core_msgs/IOComponentStatus\n## IO Component status data\nstring name            # component name\nIOStatus status        # component status\n#\n\n\n\n================================================================================\nMSG: intera_core_msgs/IOStatus\n## IO status data\n#\nstring tag             # one of the values listed below\n#   down     Inoperative, not fully instantiated\n#   ready    OK, fully operational\n#   busy     OK, not ready to output data; input data value may be stale\n#   unready  OK, not operational; data is invalid\n#   error    Error, not operational\nstring DOWN      = down\nstring READY     = ready\nstring BUSY      = busy\nstring UNREADY   = unready\nstring ERROR     = error\n#\nstring id             # message id, for internationalization\n#\nstring detail         # optional additional status detail\n#\n\n================================================================================\nMSG: intera_core_msgs/IODataStatus\n## IO Data Status\nstring name       # IO Data Component name\nstring format     # data format:\n                  # A JSON object containing one or more of the following fields:\n                  # Required:\n                  #   \"type\" : \"<type>\"  JSON Type, one of:\n                  #                      \"bool\", \"int\", \"float\", \"string\", \"object\", \"array\"\n                  #\n                  # Optional:\n                  #   \"dimensions\" : [ N,...] if absent, [ 1 ] is assumed.\n                  #   \"role\"       : Signal: \"input\", \"output\"; Port:  \"sink\", \"source\"\n                  #   \"data_type\"  : qualifier for the JSON type, may be device-specific\n                  #                  int data_types:  \"uN\", \"sN\", \"NqM\"\n                  #   other device-specific qualifiers may be used and will be passed through.\nstring data       # JSON data value.  An Array is expected, for example \"[ 0 ]\"\nIOStatus status   # Data Component status\n")]
        public class IODeviceStatus : ROSMsg
        {
            public string _type => "intera_core_msgs/IODeviceStatus";
            public string _md5sum => "0d0c97a2d700848e7ad46e09a77cf896";
            public string _full_text => "## IO Device status\ntime time                  # time the message was created\nIOComponentStatus device   # device status data\nIODataStatus[]    ports    # Ports status\nIODataStatus[]    signals  # Signals status\ntime[]            commands # recent command timestamps, for syncing\nstring[]          responses # recent command responses\n\n\n================================================================================\nMSG: intera_core_msgs/IOComponentStatus\n## IO Component status data\nstring name            # component name\nIOStatus status        # component status\n#\n\n\n\n================================================================================\nMSG: intera_core_msgs/IOStatus\n## IO status data\n#\nstring tag             # one of the values listed below\n#   down     Inoperative, not fully instantiated\n#   ready    OK, fully operational\n#   busy     OK, not ready to output data; input data value may be stale\n#   unready  OK, not operational; data is invalid\n#   error    Error, not operational\nstring DOWN      = down\nstring READY     = ready\nstring BUSY      = busy\nstring UNREADY   = unready\nstring ERROR     = error\n#\nstring id             # message id, for internationalization\n#\nstring detail         # optional additional status detail\n#\n\n================================================================================\nMSG: intera_core_msgs/IODataStatus\n## IO Data Status\nstring name       # IO Data Component name\nstring format     # data format:\n                  # A JSON object containing one or more of the following fields:\n                  # Required:\n                  #   \"type\" : \"<type>\"  JSON Type, one of:\n                  #                      \"bool\", \"int\", \"float\", \"string\", \"object\", \"array\"\n                  #\n                  # Optional:\n                  #   \"dimensions\" : [ N,...] if absent, [ 1 ] is assumed.\n                  #   \"role\"       : Signal: \"input\", \"output\"; Port:  \"sink\", \"source\"\n                  #   \"data_type\"  : qualifier for the JSON type, may be device-specific\n                  #                  int data_types:  \"uN\", \"sN\", \"NqM\"\n                  #   other device-specific qualifiers may be used and will be passed through.\nstring data       # JSON data value.  An Array is expected, for example \"[ 0 ]\"\nIOStatus status   # Data Component status\n";
            public ROSTime time = default;
            public intera_core_msgs.IOComponentStatus device = default;
            public intera_core_msgs.IODataStatus[] ports = new intera_core_msgs.IODataStatus[0];
            public intera_core_msgs.IODataStatus[] signals = new intera_core_msgs.IODataStatus[0];
            public ROSTime[] commands = new ROSTime[0];
            public string[] responses = new string[0];
            public static IODeviceStatus ROSRead(BinaryReader reader)
            {
                var o = new IODeviceStatus();
                o.time = rosmsg_builtin_util.read_ROSTime(reader);
                o.device = intera_core_msgs.IOComponentStatus.ROSRead(reader);
                o.ports = intera_core_msgs.IODataStatus.ROSReadArray(reader, -1);
                o.signals = intera_core_msgs.IODataStatus.ROSReadArray(reader, -1);
                o.commands = rosmsg_builtin_util.read_ROSTime_array(reader, -1);
                o.responses = rosmsg_builtin_util.read_string_array(reader, -1);
                return o;
            }
            public static IODeviceStatus[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new IODeviceStatus[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, IODeviceStatus msg)
            {
                rosmsg_builtin_util.write_ROSTime(writer, msg.time);
                intera_core_msgs.IOComponentStatus.ROSWrite(writer, msg.device);
                intera_core_msgs.IODataStatus.ROSWriteArray(writer, msg.ports, -1);
                intera_core_msgs.IODataStatus.ROSWriteArray(writer, msg.signals, -1);
                rosmsg_builtin_util.write_ROSTime_array(writer, msg.commands, -1);
                rosmsg_builtin_util.write_string_array(writer, msg.responses, -1);
            }
            public static void ROSWriteArray(BinaryWriter writer, IODeviceStatus[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/IONodeStatus", "260fce3c02f43bd977c92642b3c09c1d", "# IO Node Status\ntime time                    # time the message was created\nIOComponentStatus node       # IO Node status\nIOComponentStatus[] devices  # status of IO Devices in this node\ntime[] commands              # recent command timestamps, for syncing\n================================================================================\nMSG: intera_core_msgs/IOComponentStatus\n## IO Component status data\nstring name            # component name\nIOStatus status        # component status\n#\n\n\n\n================================================================================\nMSG: intera_core_msgs/IOStatus\n## IO status data\n#\nstring tag             # one of the values listed below\n#   down     Inoperative, not fully instantiated\n#   ready    OK, fully operational\n#   busy     OK, not ready to output data; input data value may be stale\n#   unready  OK, not operational; data is invalid\n#   error    Error, not operational\nstring DOWN      = down\nstring READY     = ready\nstring BUSY      = busy\nstring UNREADY   = unready\nstring ERROR     = error\n#\nstring id             # message id, for internationalization\n#\nstring detail         # optional additional status detail\n#\n")]
        public class IONodeStatus : ROSMsg
        {
            public string _type => "intera_core_msgs/IONodeStatus";
            public string _md5sum => "260fce3c02f43bd977c92642b3c09c1d";
            public string _full_text => "# IO Node Status\ntime time                    # time the message was created\nIOComponentStatus node       # IO Node status\nIOComponentStatus[] devices  # status of IO Devices in this node\ntime[] commands              # recent command timestamps, for syncing\n================================================================================\nMSG: intera_core_msgs/IOComponentStatus\n## IO Component status data\nstring name            # component name\nIOStatus status        # component status\n#\n\n\n\n================================================================================\nMSG: intera_core_msgs/IOStatus\n## IO status data\n#\nstring tag             # one of the values listed below\n#   down     Inoperative, not fully instantiated\n#   ready    OK, fully operational\n#   busy     OK, not ready to output data; input data value may be stale\n#   unready  OK, not operational; data is invalid\n#   error    Error, not operational\nstring DOWN      = down\nstring READY     = ready\nstring BUSY      = busy\nstring UNREADY   = unready\nstring ERROR     = error\n#\nstring id             # message id, for internationalization\n#\nstring detail         # optional additional status detail\n#\n";
            public ROSTime time = default;
            public intera_core_msgs.IOComponentStatus node = default;
            public intera_core_msgs.IOComponentStatus[] devices = new intera_core_msgs.IOComponentStatus[0];
            public ROSTime[] commands = new ROSTime[0];
            public static IONodeStatus ROSRead(BinaryReader reader)
            {
                var o = new IONodeStatus();
                o.time = rosmsg_builtin_util.read_ROSTime(reader);
                o.node = intera_core_msgs.IOComponentStatus.ROSRead(reader);
                o.devices = intera_core_msgs.IOComponentStatus.ROSReadArray(reader, -1);
                o.commands = rosmsg_builtin_util.read_ROSTime_array(reader, -1);
                return o;
            }
            public static IONodeStatus[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new IONodeStatus[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, IONodeStatus msg)
            {
                rosmsg_builtin_util.write_ROSTime(writer, msg.time);
                intera_core_msgs.IOComponentStatus.ROSWrite(writer, msg.node);
                intera_core_msgs.IOComponentStatus.ROSWriteArray(writer, msg.devices, -1);
                rosmsg_builtin_util.write_ROSTime_array(writer, msg.commands, -1);
            }
            public static void ROSWriteArray(BinaryWriter writer, IONodeStatus[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/IOStatus", "a8daeb84c9abffc88ad8ca636f5fd8a0", "## IO status data\n#\nstring tag             # one of the values listed below\n#   down     Inoperative, not fully instantiated\n#   ready    OK, fully operational\n#   busy     OK, not ready to output data; input data value may be stale\n#   unready  OK, not operational; data is invalid\n#   error    Error, not operational\nstring DOWN      = down\nstring READY     = ready\nstring BUSY      = busy\nstring UNREADY   = unready\nstring ERROR     = error\n#\nstring id             # message id, for internationalization\n#\nstring detail         # optional additional status detail\n#\n")]
        public class IOStatus : ROSMsg
        {
            public string _type => "intera_core_msgs/IOStatus";
            public string _md5sum => "a8daeb84c9abffc88ad8ca636f5fd8a0";
            public string _full_text => "## IO status data\n#\nstring tag             # one of the values listed below\n#   down     Inoperative, not fully instantiated\n#   ready    OK, fully operational\n#   busy     OK, not ready to output data; input data value may be stale\n#   unready  OK, not operational; data is invalid\n#   error    Error, not operational\nstring DOWN      = down\nstring READY     = ready\nstring BUSY      = busy\nstring UNREADY   = unready\nstring ERROR     = error\n#\nstring id             # message id, for internationalization\n#\nstring detail         # optional additional status detail\n#\n";
            public string tag = default;
            public string id = default;
            public string detail = default;
            public static IOStatus ROSRead(BinaryReader reader)
            {
                var o = new IOStatus();
                o.tag = rosmsg_builtin_util.read_string(reader);
                o.id = rosmsg_builtin_util.read_string(reader);
                o.detail = rosmsg_builtin_util.read_string(reader);
                return o;
            }
            public static IOStatus[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new IOStatus[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, IOStatus msg)
            {
                rosmsg_builtin_util.write_string(writer, msg.tag);
                rosmsg_builtin_util.write_string(writer, msg.id);
                rosmsg_builtin_util.write_string(writer, msg.detail);
            }
            public static void ROSWriteArray(BinaryWriter writer, IOStatus[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
    namespace intera_core_msgs
    {
        [ROSMsgInfo("intera_core_msgs/IOComponentStatus", "7daed407477edd76573df598b0375a48", "## IO Component status data\nstring name            # component name\nIOStatus status        # component status\n#\n\n\n\n================================================================================\nMSG: intera_core_msgs/IOStatus\n## IO status data\n#\nstring tag             # one of the values listed below\n#   down     Inoperative, not fully instantiated\n#   ready    OK, fully operational\n#   busy     OK, not ready to output data; input data value may be stale\n#   unready  OK, not operational; data is invalid\n#   error    Error, not operational\nstring DOWN      = down\nstring READY     = ready\nstring BUSY      = busy\nstring UNREADY   = unready\nstring ERROR     = error\n#\nstring id             # message id, for internationalization\n#\nstring detail         # optional additional status detail\n#\n")]
        public class IOComponentStatus : ROSMsg
        {
            public string _type => "intera_core_msgs/IOComponentStatus";
            public string _md5sum => "7daed407477edd76573df598b0375a48";
            public string _full_text => "## IO Component status data\nstring name            # component name\nIOStatus status        # component status\n#\n\n\n\n================================================================================\nMSG: intera_core_msgs/IOStatus\n## IO status data\n#\nstring tag             # one of the values listed below\n#   down     Inoperative, not fully instantiated\n#   ready    OK, fully operational\n#   busy     OK, not ready to output data; input data value may be stale\n#   unready  OK, not operational; data is invalid\n#   error    Error, not operational\nstring DOWN      = down\nstring READY     = ready\nstring BUSY      = busy\nstring UNREADY   = unready\nstring ERROR     = error\n#\nstring id             # message id, for internationalization\n#\nstring detail         # optional additional status detail\n#\n";
            public string name = default;
            public intera_core_msgs.IOStatus status = default;
            public static IOComponentStatus ROSRead(BinaryReader reader)
            {
                var o = new IOComponentStatus();
                o.name = rosmsg_builtin_util.read_string(reader);
                o.status = intera_core_msgs.IOStatus.ROSRead(reader);
                return o;
            }
            public static IOComponentStatus[] ROSReadArray(BinaryReader reader, int count)
            {
                if (count < 0) count = (int)reader.ReadUInt32();
                var o = new IOComponentStatus[count];
                for (int i = 0; i < count; i++) o[i] = ROSRead(reader);
                return o;
            }
            public static void ROSWrite(BinaryWriter writer, IOComponentStatus msg)
            {
                rosmsg_builtin_util.write_string(writer, msg.name);
                intera_core_msgs.IOStatus.ROSWrite(writer, msg.status);
            }
            public static void ROSWriteArray(BinaryWriter writer, IOComponentStatus[] msg, int count)
            {
                rosmsg_builtin_util.do_write_count(writer, msg, count);
                for (int i = 0; i < msg.Length; i++) ROSWrite(writer, msg[i]);
            }
        }
    }
}
