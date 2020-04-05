"""A number of custom exceptions and errors that are used in the `panda_training` package."""

# Main python imports
from __future__ import print_function
from builtins import super


#################################################
# Custom exceptions #############################
#################################################
class InputMessageInvalid(Exception):
    """

    Attributes
    ----------
    details : dict
        Dictionary containing extra Exceptino information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the InputMessageInvalid exception object.

        Parameters
        ----------
        message : str, optional
            Exception message specifying whether the exception occured, by default "".
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
