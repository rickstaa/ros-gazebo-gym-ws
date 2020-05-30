"""A number of custom ROS errors that are used in the `panda_training` package."""


# Main python imports
import sys
from panda_training.functions import list_2_human_text

# ROS python imports
import rospy


#################################################
# Custom ROS errors #############################
#################################################
def arg_type_error_shutdown(arg_name, depth, invalid_types, valid_types):
    """This function displays a argument type invalid ROS error and shutdown the
    ROS node.

    Parameters
    ----------
    arg_name : str
        The name of the argument.
    depth : int
        The dict depth at which the error occurred.
    invalid_type : tuple
        The type that was invalid.
    valid_types : tuple
        The types that are valid.
    """

    # Throw Type error and shutdown ROS node
    if depth == 0:
        logerr_msg = (
            "Shutting down '%s' since input argument '%s' was of type %s while "
            "the PandaTaskEnv expects it to be of type %s. Please fix the type "
            "and try again."
            % (
                rospy.get_name(),
                arg_name,
                list_2_human_text(
                    ["'" + str(item.__name__) + "'" for item in invalid_types],
                    end_seperator="and",
                ),
                list_2_human_text(
                    ["'" + str(item.__name__) + "'" for item in valid_types],
                    end_seperator="or",
                ),
            )
        )
    else:
        logerr_msg = (
            "Shutting down '%s' since input argument '%s' contains items that have "
            "type %s while the PandaTaskEnv only expects %s. Please fix the type "
            "and try again."
            % (
                rospy.get_name(),
                arg_name,
                list_2_human_text(
                    ["'" + str(item.__name__) + "'" for item in invalid_types],
                    end_seperator="and",
                ),
                list_2_human_text(
                    ["'" + str(item.__name__) + "'" for item in valid_types],
                    end_seperator="or",
                ),
            )
        )
    rospy.logerr(logerr_msg)
    sys.exit(0)


def arg_keys_error_shutdown(arg_name, missing_keys, extra_keys=[]):
    """This function displays a argument keys invalid ROS error and shutdown the
    ROS node.

    Parameters
    ----------
    arg_name : str
        The name of the argument.
    missing_keys : list
        The dictionary keys that were missing from the input argument.
    extra_keys : list
        The dictionary keys that were present but should not be.
    """

    # Throw Type error and shutdown ROS node
    logerr_msg = (
        "Shutting down '%s' since input argument '%s' contains %s. Please %s and "
        "try again."
        % (
            rospy.get_name(),
            arg_name,
            (
                "missing and invalid keys"
                if len(missing_keys) > 0 and len(extra_keys) > 0
                else (
                    (
                        "several missing keys"
                        if len(missing_keys) > 1
                        else "a missing key"
                    )
                    if len(missing_keys) > 0
                    else (
                        "several invalid keys"
                        if len(missing_keys) > 1
                        else "a invalid key"
                    )
                )
            ),
            (
                "remove [%s] and add [%s]"
                % (
                    list_2_human_text(
                        ["'" + str(item) + "'" for item in extra_keys],
                        end_seperator="and",
                    ),
                    list_2_human_text(
                        ["'" + str(item) + "'" for item in missing_keys],
                        end_seperator="and",
                    ),
                )
                if len(missing_keys) > 0 and len(extra_keys) > 0
                else (
                    (
                        "remove [%s]"
                        % (
                            list_2_human_text(
                                ["'" + str(item) + "'" for item in extra_keys],
                                end_seperator="and",
                            )
                        )
                        if len(extra_keys) > 0
                        else "add [%s]"
                        % (
                            list_2_human_text(
                                ["'" + str(item) + "'" for item in missing_keys],
                                end_seperator="and",
                            )
                        )
                    )
                )
            ),
        )
    )
    rospy.logerr(logerr_msg)
    sys.exit(0)


def arg_value_error_shutdown(arg_name, invalid_values, valid_values):
    """This function displays a value invalid ROS error and shutdown the ROS node.

    Parameters
    ----------
    arg_name : str
        The name of the argument.
    invalid_values : list
        The values that were invalid.
    extra_keys : list
        A list of valid values.
    """
    logerr_msg = (
        "Shutting down '%s' since input argument '%s' contains %s '%s'. Valid values "
        "for the '%s' input argument are [%s]"
        % (
            rospy.get_name(),
            arg_name,
            "a invalid value" if len(invalid_values) else "invalid values",
            list_2_human_text(
                ["'" + str(item.__name__) + "'" for item in invalid_values],
                end_seperator="and",
            ),
            arg_name,
            list_2_human_text(
                ["'" + str(item.__name__) + "'" for item in valid_values],
                end_seperator="and",
            ),
        )
    )
    rospy.logerr(logerr_msg)
    sys.exit(0)


if __name__ == "__main__":
    arg_type_error_shutdown(
        arg_name="beerend",
        depth=1,
        invalid_types=[int, bool],
        valid_types=[float, dict],
    )
    print("tests")
