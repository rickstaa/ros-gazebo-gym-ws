"""Several functions that are used in the 'panda_training' scripts."""

# Main python imports
import time
import os
import re
import shutil
import glob

# ROS python imports
import rospy


#################################################
# Functions #####################################
#################################################
def find_lowest_positive_missing_no(input_list):
    """Function finds and returns the lowest missing number in a list.

    Parameters
    ----------
    input_list : list
        An input list of int's.

    Returns
    -------
    int
        The lowest number that is missing from the list.
    """

    # Validate whether al list items are integers
    if not all([type(item) == int for item in input_list]):
        KeyError("Input list contains non-integer items.")

    # to store next array element in
    # current traversal
    list_length = len(input_list)
    for ii in range(list_length):

        # if value is negative or greater
        # than array size, then it cannot
        # be marked in array. So move to
        # next element.
        if input_list[ii] <= 0 or input_list[ii] > list_length:
            continue

        val = input_list[ii]

        # traverse the array until we
        # reach at an element which
        # is already marked or which
        # could not be marked.
        while input_list[val - 1] != val:
            nextval = input_list[val - 1]
            input_list[val - 1] = val
            val = nextval
            if val <= 0 or val > list_length:
                break

    # find first array index which is
    # not marked which is also the
    # smallest positive missing
    # number.
    for ii in range(list_length):
        if input_list[ii] != ii + 1:
            return ii + 1

    # if all indices are marked, then
    # smallest missing positive
    # number is array_size + 1.
    return list_length + 1


def get_unique_file_suffix(folder_path=None, prefix_type="timestamp"):
    """Function used to create an unique file identifier suffix.

    Parameters
    ----------
    folder_path : str, optional
        The folder in which you want to save the model, by default None.
    prefix_type : str, optional
        The type of suffix you want to use i.e. 'timestamp' or 'number', by default
        "timestamp".

    Raises
    ------
    TypeError
        If the prefix_type does not exists.
    """

    # Validate type variable
    if prefix_type.lower() not in ["timestamp", "number"]:
        raise TypeError(
            "Prefix %s is not valid. Valid values are 'timestamp' and 'number'."
            % prefix_type
        )

    # Create an identifier suffix based on the suffix type
    if prefix_type == "timestamp":
        suffix = int(time.time())
        return str(suffix)
    else:  # If number

        # Check if folder was supplied
        if not folder_path:
            rospy.logwarn(
                "Unique number identifier could not be created as no folder path was "
                "given. Timestamp used instead,"
            )
            suffix = int(time.time())
            return str(suffix)

        # Check if folder exists
        abs_path = os.path.abspath(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), folder_path)
        )
        if not os.path.isdir(abs_path):
            raise FileNotFoundError(abs_path)

        # Find the lowest missing number suffix in the folder_path
        sub_dirs = [d for d in os.listdir(folder_path)]  # Find all subfolders
        sub_dirs_splitted = [
            item.split("-")[-1] for item in sub_dirs
        ]  # Split base on - delimiter
        sub_dirs_filtered = [re.sub("[^0-9]", "", item) for item in sub_dirs_splitted]
        suffix_list = [int(item) for item in sub_dirs_filtered if item]
        suffix = find_lowest_positive_missing_no(suffix_list)
        return str(suffix)


def move_all_files_in_dir(srcDir, dstDir):
    """Function used to move all files in a given directory to a new directory.

    Parameters
    ----------
    srcDir : st
        The path of the source directory.
    dstDir : str
        The path of the destination directory.
    """

    # Check if both the are directories
    if os.path.isdir(srcDir) and os.path.isdir(dstDir):
        # Iterate over all the files in source directory
        for filePath in glob.glob(os.path.join(srcDir, "*")):
            # Move each file to destination Directory
            shutil.move(filePath, dstDir)
    else:
        print("srcDir & dstDir should be Directories")


def backup_model(model_file_path, backup_model=True, backup_checkpoints=False):
    """Backups a RL model together with its checkpoints.

    Parameters
    ----------
    model_file_path : str
        The path of the RL model.
    backup_model : bool, optional
        Whether to backup the RL model, by default True
    backup_checkpoints : bool, optional
        Whether to also backup corresponding checkpoints data, by default False
    """

    # Retrieve model name, path and create backup path
    backup_folder = os.path.abspath(
        os.path.join(model_file_path, os.pardir, "backups")
    )  # Create backup folder path
    model_name = os.path.basename(model_file_path).split(".")[0]  # Retrieve model name
    backup_file_name = (
        model_name + "-" + get_unique_file_suffix()
    )  # Create backup file name
    backup_file_path = os.path.abspath(
        os.path.join(backup_folder, backup_file_name + ".zip")
    )  # Get the backup file path

    # Create model backup folder if it does not yet exist
    if any([backup_model, backup_checkpoints]):
        if not os.path.isdir(backup_folder):
            os.makedirs(backup_folder)
            print(
                "Created model backup folder '%s' as it did not yet exits."
                % backup_folder
            )

    # Backup the RL model
    rospy.loginfo("Creating backup of the current model: %s" % model_file_path)
    if backup_model:
        shutil.move(model_file_path, backup_file_path)

    # Backup the model checkpoints if requested and available
    if backup_checkpoints:
        checkpoints_dir = os.path.abspath(
            os.path.join(
                os.path.join(model_file_path, os.pardir), "checkpoints", model_name
            )
        )  # Retrieve checkpoints path

        # Check if checkpoints are present
        if os.path.isdir(checkpoints_dir):  # If exists
            if (
                len(glob.glob(os.path.join(checkpoints_dir, "*steps.zip"))) > 0
            ):  # If contains files
                checkpoints_dst = os.path.join(
                    backup_folder, "checkpoints", backup_file_name
                )  # Create checkpoints backup path

                # Create model backup folder if it does not yet exist
                if not os.path.isdir(checkpoints_dst):
                    os.makedirs(checkpoints_dst)
                    print(
                        "Created checkpoints backup folder '%s' as it did not yet "
                        "exits." % checkpoints_dst
                    )

                # Backup the checkpoints
                move_all_files_in_dir(checkpoints_dir, checkpoints_dst)
