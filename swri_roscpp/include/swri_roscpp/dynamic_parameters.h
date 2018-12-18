#ifndef SWRI_ROSCPP_DYNAMIC_PARAMETERS_H_
#define SWRI_ROSCPP_DYNAMIC_PARAMETERS_H_

#include <map>
#include <string>

#include <boost/thread/mutex.hpp>

#include <ros/console.h>
#include <ros/node_handle.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/GroupState.h>
#include <dynamic_reconfigure/Reconfigure.h>

namespace swri
{
  struct DynamicValue
  {
    enum Type
    {
      Bool = 0,
      Float = 1,
      Double = 2,
      Int = 3,
      String = 4
    };

    Type type;
    std::string name;
    std::string description;

    void* pointer;//pointer to the parameter to update on change

    // Defaults, maximum and minimum values for this parameter
    union
    {
      double d;
      bool b;
      int i;
    } Default;
    union
    {
      double d;
      int i;
    } Min;
    union
    {
      double d;
      int i;
    } Max;
    std::string default_string;// to get around union issues with strings
  };

  class DynamicParameters
  {
    ros::Publisher descr_pub_;
    ros::Publisher update_pub_;
    ros::ServiceServer set_service_;
    ros::NodeHandle nh_;

    std::map<std::string, DynamicValue> values_;

    boost::mutex mutex_;

    bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
          dynamic_reconfigure::Reconfigure::Response &rsp)
    {
      ROS_INFO("Got configuration change request");

      boost::mutex::scoped_lock lock(mutex_);
      
      // update the parameters
      for (int i = 0; i < req.config.doubles.size(); i++)
      {
        dynamic_reconfigure::DoubleParameter param = req.config.doubles[i];
        std::map<std::string, DynamicValue>::iterator iter = values_.find(param.name);
        if (iter == values_.end())
        {
          ROS_ERROR("Could not find parameter '%s'", param.name.c_str());
          continue;
        }

        if (iter->second.type != DynamicValue::Double && iter->second.type != DynamicValue::Float)
        {
          ROS_ERROR("Value '%s' was not a double type.", param.name.c_str());
          continue;
        }

        if (iter->second.type == DynamicValue::Double)
        {
          double* v = (double*)iter->second.pointer;
          *v = param.value;
        }

        if (iter->second.type == DynamicValue::Float)
        {
          float* v = (float*)iter->second.pointer;
          *v = (float)param.value;
        }
      }

      for (int i = 0; i < req.config.ints.size(); i++)
      {
        dynamic_reconfigure::IntParameter param = req.config.ints[i];
        std::map<std::string, DynamicValue>::iterator iter = values_.find(param.name);
        if (iter == values_.end())
        {
          ROS_ERROR("Could not find parameter '%s'", param.name.c_str());
          continue;
        }

        if (iter->second.type != DynamicValue::Int)
        {
          ROS_ERROR("Value '%s' was not a int type.", param.name.c_str());
          continue;
        }

        int* v = (int*)iter->second.pointer;
        *v = param.value;
      }

      for (int i = 0; i < req.config.bools.size(); i++)
      {
        dynamic_reconfigure::BoolParameter param = req.config.bools[i];
        std::map<std::string, DynamicValue>::iterator iter = values_.find(param.name);
        if (iter == values_.end())
        {
          ROS_ERROR("Could not find parameter '%s'", param.name.c_str());
          continue;
        }

        if (iter->second.type != DynamicValue::Bool)
        {
          ROS_ERROR("Value '%s' was not a bool type.", param.name.c_str());
          continue;
        }

        bool* v = (bool*)iter->second.pointer;
        *v = param.value;
      }

      for (int i = 0; i < req.config.strs.size(); i++)
      {
        dynamic_reconfigure::StrParameter param = req.config.strs[i];
        std::map<std::string, DynamicValue>::iterator iter = values_.find(param.name);
        if (iter == values_.end())
        {
          ROS_ERROR("Could not find parameter '%s'", param.name.c_str());
          continue;
        }

        if (iter->second.type != DynamicValue::String)
        {
          ROS_ERROR("Value '%s' was not a string type.", param.name.c_str());
          continue;
        }

        std::string* v = (std::string*)iter->second.pointer;
        *v = param.value;
      }

      updateCurrent(rsp.config);

      return true;
    }

    // Updates a config with the current parameter values
    void updateCurrent(dynamic_reconfigure::Config& config)
    {
      for (std::map<std::string, DynamicValue>::iterator value = values_.begin(); value != values_.end(); value++)
      {
        if (value->second.type == DynamicValue::Double)
        {
          dynamic_reconfigure::DoubleParameter param;
          param.name = value->first;
          param.value = *(double*)value->second.pointer;
          config.doubles.push_back(param);
        }
        else if (value->second.type == DynamicValue::Float)
        {
          dynamic_reconfigure::DoubleParameter param;
          param.name = value->first;
          param.value = *(float*)value->second.pointer;
          config.doubles.push_back(param);
        }
        else if (value->second.type == DynamicValue::Int)
        {
          dynamic_reconfigure::IntParameter param;
          param.name = value->first;
          param.value = *(int*)value->second.pointer;
          config.ints.push_back(param);
        }
        else if (value->second.type == DynamicValue::Bool)
        {
          dynamic_reconfigure::BoolParameter param;
          param.name = value->first;
          param.value = *(bool*)value->second.pointer;
          config.bools.push_back(param);
        }
        else if (value->second.type == DynamicValue::String)
        {
          dynamic_reconfigure::StrParameter param;
          param.name = value->first;
          param.value = *(std::string*)value->second.pointer;
          config.strs.push_back(param);
        }
      }

      dynamic_reconfigure::GroupState gs;
      gs.name = "Default";
      gs.state = true;
      gs.id = 0;
      gs.parent = 0;
      config.groups.push_back(gs);
      update_pub_.publish(config);
    }

    public:
    
    // Sets up the node handle and publishers. Be sure to call this before finalize or any of the 'get*' calls.
    void initialize(ros::NodeHandle& pnh)
    {
      boost::mutex::scoped_lock lock(mutex_);
      nh_ = pnh;

      descr_pub_ = nh_.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
      update_pub_ = nh_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);

      set_service_ = nh_.advertiseService("set_parameters",
            &DynamicParameters::setConfigCallback, this);
    }

    // Publishes the configuration parameters that have been added
    void finalize()
    {
      // publish the configs as one group
      dynamic_reconfigure::ConfigDescription rdesc;
      dynamic_reconfigure::Group group;
      group.name = "Default";
      group.type = "";
      group.id = 0;
      group.parent = 0;

      dynamic_reconfigure::GroupState gs;
      gs.name = "Default";
      gs.state = true;
      gs.id = 0;
      gs.parent = 0;
      for (std::map<std::string, DynamicValue>::iterator param = values_.begin(); param != values_.end(); param++)
      {
        std::string type;
        if (param->second.type == DynamicValue::Bool)
        {
          type = "bool";

          dynamic_reconfigure::BoolParameter desc;
          desc.name = param->first;
          desc.value = true;
          rdesc.max.bools.push_back(desc);
          desc.value = false;
          rdesc.min.bools.push_back(desc);
          desc.value = param->second.Default.b;
          rdesc.dflt.bools.push_back(desc);
        }
        else if (param->second.type == DynamicValue::Float)
        {
          type = "double";

          dynamic_reconfigure::DoubleParameter desc;
          desc.name = param->first;
          desc.value = param->second.Max.d;
          rdesc.max.doubles.push_back(desc);
          desc.value = param->second.Min.d;
          rdesc.min.doubles.push_back(desc);
          desc.value = param->second.Default.d;
          rdesc.dflt.doubles.push_back(desc);
        }
        else if (param->second.type == DynamicValue::Double)
        {
          type = "double";

          dynamic_reconfigure::DoubleParameter desc;
          desc.name = param->first;
          desc.value = param->second.Max.d;
          rdesc.max.doubles.push_back(desc);
          desc.value = param->second.Min.d;
          rdesc.min.doubles.push_back(desc);
          desc.value = param->second.Default.d;
          rdesc.dflt.doubles.push_back(desc);
        }
        else if (param->second.type == DynamicValue::Int)
        {
          type = "int";

          dynamic_reconfigure::IntParameter desc;
          desc.name = param->first;
          desc.value = param->second.Max.i;
          rdesc.max.ints.push_back(desc);
          desc.value = param->second.Min.i;
          rdesc.min.ints.push_back(desc);
          desc.value = param->second.Default.i;
          rdesc.dflt.ints.push_back(desc);
        }
        else if (param->second.type == DynamicValue::String)
        {
          type = "string";
          dynamic_reconfigure::StrParameter desc;
          desc.name = param->first;
          desc.value = "";
          rdesc.max.strs.push_back(desc);
          desc.value = "";
          rdesc.min.strs.push_back(desc);
          desc.value = param->second.default_string;
          rdesc.dflt.strs.push_back(desc);
        }

        dynamic_reconfigure::ParamDescription desc;
        desc.name = param->first;
        desc.type = type;
        desc.level = 0;
        desc.description = param->second.description;
        desc.edit_method = "";
        group.parameters.push_back(desc);
      }
      rdesc.max.groups.push_back(gs);
      rdesc.min.groups.push_back(gs);
      rdesc.dflt.groups.push_back(gs);
      rdesc.groups.push_back(group);
      descr_pub_.publish(rdesc);

      dynamic_reconfigure::Config config;
      updateCurrent(config);
    }

    boost::mutex& mutex()
    {
      return mutex_;
    }

    inline
    void get(const std::string &name,
      float &variable,
      const float default_value,
      const std::string description = "None.",
      const float min = -100,
      const float max = 100)
    {
      DynamicValue value;
      value.type = DynamicValue::Float;
      value.description = description;
      value.Min.d = min;
      value.Max.d = max;
      value.Default.d = default_value;
      value.pointer = (void*)&variable;
      values_[name] = value;

      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, variable, default_value);
      ROS_INFO("Read dynamic parameter %s = %f", name.c_str(), variable);
    }
    
    inline
    void get(const std::string &name,
      double &variable,
      const double default_value,
      const std::string description = "None.",
      const double min = -100,
      const double max = 100)
    {
      DynamicValue value;
      value.type = DynamicValue::Double;
      value.description = description;
      value.Min.d = min;
      value.Max.d = max;
      value.Default.d = default_value;
      value.pointer = (void*)&variable;
      values_[name] = value;
 
      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, variable, default_value);
      ROS_INFO("Read dynamic parameter %s = %lf", name.c_str(), variable);
    }

    inline
    void get(const std::string &name,
      int &variable,
      const int default_value,
      const std::string description = "None.",
      const int min = -100,
      const int max = 100)
    {
      DynamicValue value;
      value.type = DynamicValue::Int;
      value.description = description;
      value.Min.i = min;
      value.Max.i = max;
      value.Default.i = default_value;
      value.pointer = (void*)&variable;
      values_[name] = value;
 
      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, variable, default_value);
      ROS_INFO("Read dynamic parameter %s = %i", name.c_str(), variable);
    }

    inline
    void get(const std::string &name,
      bool &variable,
      const bool default_value,
      const std::string description = "None.")
    {
      DynamicValue value;
      value.type = DynamicValue::Bool;
      value.description = description;
      value.Default.b = default_value;
      value.pointer = (void*)&variable;
      values_[name] = value;
 
      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, variable, default_value);
      ROS_INFO("Read dynamic parameter %s = %s", name.c_str(), variable ? "true" : "false");
    }

    inline
    void get(const std::string &name,
      std::string &variable,
      const std::string default_value,
      const std::string description = "None.")
    {
      DynamicValue value;
      value.type = DynamicValue::Bool;
      value.description = description;
      value.default_string = default_value;
      value.pointer = (void*)&variable;
      values_[name] = value;
 
      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, variable, default_value);
      ROS_INFO("Read dynamic parameter %s = %s", name.c_str(), variable.c_str());
    }
  };
}  // namespace swri_param
#endif  // SWRI_ROSCPP_DYNAMIC_PARAMETERS_H_
