#------------------------------------------------------------------------------------------------------------
# File: scripts/command_dict.py
# Description: This file contains the dictionary of commands and resource types used in the system.
#              This file is used by log_writer.py
# Author: Balachandra Bhat (github.com/bnbhat)
# Date: 2023-08-03
# Version: 1.0
#------------------------------------------------------------------------------------------------------------

##<IMPORTANT># UPDATE THE DICTIONARY BELOW WITH THE LATEST COMMANDS AND RESOURCE TYPES #<IMPORTANT>##

resource_type_dict = {
    0 : 'ROBOT',
    1 : 'HUMAN'
}

status_code_dict = {
    80: 'NOT_IN_PROGRESS',
    10: 'RUNNING',
    1: 'STOP',
    2: 'PAUSE',
    3: 'START',
    4: 'STEP_FORWARD',
    5: 'STEP_BACKWARDS',
    30: 'HOME_POSITION_ROBOT',
    60: 'FAILURE',
    70: 'FINISH',
    90: 'READY_ROBOT',
    95: 'ASSEMBLY_FINISHED',
    -1: 'NOT_STARTED',
    100: 'WAITING',  # Deprecated
    200: 'RECEIVED',  # Deprecated
    30: 'G_IDLE',
    31: 'G_REDO_STEP',
    33: 'G_STEP_BACKWARDS',
    35: 'G_START_ASSEMBLY',
    39: 'G_STOP_ROBOT',
    36: 'G_PAUSE_ROBOT',
    37: 'G_RESUME_ROBOT',
    38: 'G_FINISH_HUMAN',
    40: 'G_REPLY_AUDIO_MSG',
    50: 'G_NOTIFY_FAILURE',
}

def get_resource_type(code):
    try:
        return resource_type_dict[code]
    except:
        return code

def get_status_code(code):
    try:
        return status_code_dict[code]
    except:
        return code