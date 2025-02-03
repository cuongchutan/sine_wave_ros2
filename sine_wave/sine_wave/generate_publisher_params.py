# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
import rclpy.parameter
from generate_parameter_library_py.python_validators import ParameterValidators

import generate_parameter_module_example.custom_validation as custom_validators

import yaml
import os

pkg_install_dir = os.path.join(os.getenv('PWD'), 'install', 'sine_wave')
cfg_dir = os.path.join(pkg_install_dir, 'share', 'sine_wave', 'config')

class sine_wave:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        # Load parameters from YAML file
        yaml_path = os.path.join(cfg_dir, 'params.yaml')
        with open(yaml_path, 'r') as file:
            params = yaml.safe_load(file)
        publisher_frequency = params['sine_wave']['publisher_frequency']['default_value']
        amplitude = params['sine_wave']['amplitude']['default_value']
        angular_frequency = params['sine_wave']['angular_frequency']['default_value']
        phase = params['sine_wave']['phase']['default_value']



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = sine_wave.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("sine_wave." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def unpack_parameter_dict(self, namespace: str, parameter_dict: dict):
            """
            Flatten a parameter dictionary recursively.

            :param namespace: The namespace to prepend to the parameter names.
            :param parameter_dict: A dictionary of parameters keyed by the parameter names
            :return: A list of rclpy Parameter objects
            """
            parameters = []
            for param_name, param_value in parameter_dict.items():
                full_param_name = namespace + param_name
                # Unroll nested parameters
                if isinstance(param_value, dict):
                    nested_params = self.unpack_parameter_dict(
                            namespace=full_param_name + rclpy.parameter.PARAMETER_SEPARATOR_STRING,
                            parameter_dict=param_value)
                    parameters.extend(nested_params)
                else:
                    parameters.append(rclpy.parameter.Parameter(full_param_name, value=param_value))
            return parameters

        def set_params_from_dict(self, param_dict):
            params_to_set = self.unpack_parameter_dict('', param_dict)
            self.update(params_to_set)

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters


        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "publisher_frequency":
                    updated_params.publisher_frequency = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "amplitude":
                    updated_params.amplitude = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "angular_frequency":
                    updated_params.angular_frequency = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "phase":
                    updated_params.phase = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "publisher_frequency"):
                descriptor = ParameterDescriptor(description="The frequency to publish data of the Publisher", read_only = True)
                parameter = updated_params.publisher_frequency
                self.node_.declare_parameter(self.prefix_ + "publisher_frequency", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "amplitude"):
                descriptor = ParameterDescriptor(description="The amplitude of a sine wave", read_only = True)
                parameter = updated_params.amplitude
                self.node_.declare_parameter(self.prefix_ + "amplitude", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "angular_frequency"):
                descriptor = ParameterDescriptor(description="The angular frequency of a sine wave", read_only = True)
                parameter = updated_params.angular_frequency
                self.node_.declare_parameter(self.prefix_ + "angular_frequency", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "phase"):
                descriptor = ParameterDescriptor(description="The initial phase of a sine wave", read_only = True)
                parameter = updated_params.phase
                self.node_.declare_parameter(self.prefix_ + "phase", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "publisher_frequency")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.publisher_frequency = param.value
            param = self.node_.get_parameter(self.prefix_ + "amplitude")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.amplitude = param.value
            param = self.node_.get_parameter(self.prefix_ + "angular_frequency")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.angular_frequency = param.value
            param = self.node_.get_parameter(self.prefix_ + "phase")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.phase = param.value


            self.update_internal_params(updated_params)
