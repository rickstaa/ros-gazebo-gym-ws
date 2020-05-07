"""A number of custom exceptions and errors that are used in the `panda_training` package."""

# Main python imports
from __future__ import print_function
from builtins import super


#################################################
# Custom exceptions #############################
#################################################
class InputMessageInvalid(Exception):
    """Custom exception that is raised when an input argument to one of the
    panda_training functions is invalid.

    Attributes
    ----------
    details : dict
        Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the InputMessageInvalid exception object.

        Parameters
        ----------
        message : str, optional
            Exception message specifying whether the exception occurred, by default "".
        log_message : str, optional
            Full log message, by default "".
        details : dict
            Additional dictionary that can be used to supply the user with more details
            about why the exception occurred.
        """

        # Call the base class constructor with the parameters it needs
        super().__init__(message)

        # Set attributes
        self.log_message = log_message
        self.details = details


class EePoseLookupError(Exception):
    """Custom exception that is raised when an error occurred while trying to retrieve
    the EE pose.

    Attributes
    ----------
    details : dict
        Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the EePoseLookupError exception object.

        Parameters
        ----------
        message : str, optional
            Exception message specifying whether the exception occurred, by default "".
        log_message : str, optional
            Full log message, by default "".
        details : dict
            Additional dictionary that can be used to supply the user with more details
            about why the exception occurred.
        """

        # Call the base class constructor with the parameters it needs
        super().__init__(message)

        # Set attributes
        self.log_message = log_message
        self.details = details


class EeRpyLookupError(Exception):
    """Custom exception that is raised when an error occurred while trying to retrieve
    the EE orientation (given in euler angles).

    Attributes
    ----------
    details : dict
        Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the EePoseLookupError exception object.

        Parameters
        ----------
        message : str, optional
            Exception message specifying whether the exception occurred, by default "".
        log_message : str, optional
            Full log message, by default "".
        details : dict
            Additional dictionary that can be used to supply the user with more details
            about why the exception occurred.
        """

        # Call the base class constructor with the parameters it needs
        super().__init__(message)

        # Set attributes
        self.log_message = log_message
        self.details = details
