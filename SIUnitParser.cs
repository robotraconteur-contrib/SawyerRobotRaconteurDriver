using System;
using System.Collections.Generic;
using System.Text;

namespace SawyerRobotRaconteurDriver
{
    public class YamlSIUnit
    {
        public string value { get; set; }
        public static explicit operator YamlSIUnit(string s) => new YamlSIUnit() { value = s };
    }
}
