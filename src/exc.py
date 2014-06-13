#!/usr/bin/env python
# coding:utf8

"""
@package exc

Exceptions package.

It provides exception classes for the events regarding the hand tracking and 
communication between server and client.

Created on 17/04/2014
@author Óscar Gómez <oscar.gomez@uji.es>
"""

class QuitMessageException(Exception):
    """
    Exception raised by a hand tracking server or client whenever a 'QUIT' 
    message is received during communication with the other side.
    """
    def __init__(self, msg):
        """
        Constructor.
        
        @param msg The message to be shown upon processing the exception
        """
        super(QuitMessageException, self).__init__(msg)