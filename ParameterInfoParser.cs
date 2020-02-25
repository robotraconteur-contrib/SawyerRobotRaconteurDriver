using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace SawyerRobotRaconteurDriver
{
    public class YamlParameterInfo
    {
        public YamlIdentifier parameter_identifier { get; set; }
        public YamlDeviceClass parameter_class { get; set; }
        public YamlDataType data_type { get; set; }
        public string user_description { get; set; }
        public YamlVarValue default_value { get; set; }
        public YamlVarValue min_value { get; set; }
        public YamlVarValue max_value { get; set; }
        public Dictionary<string,YamlVarValue> enum_values { get; set; }
        public Dictionary<string, YamlVarValue> extended { get; set; }

        public void CopyTo(com.robotraconteur.param.ParameterInfo info)
        {
            info.parameter_identifier = parameter_identifier?.ToRRInfo();
            info.parameter_class = parameter_class?.ToRRInfo();
            info.data_type = data_type?.ToRRInfo();
            info.user_description = user_description ?? "";
            info.default_value = default_value?.value;
            info.min_value = min_value?.value;
            info.max_value = max_value?.value;
            info.enum_values = enum_values?.ToDictionary(x => x.Key, x => x.Value?.value);
            info.extended = extended?.ToDictionary(x => x.Key, x => x.Value?.value);
        }

        public com.robotraconteur.param.ParameterInfo ToRRInfo()
        {
            var info = new com.robotraconteur.param.ParameterInfo();
            CopyTo(info);
            return info;
        }

    }
}
