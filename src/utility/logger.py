"""
Description of what this file does.
"""

# logging setup
import logging
from functools import partial

logging.NOTE = logging.INFO - 5
logging.addLevelName(logging.NOTE, 'NOTE')
logging.basicConfig()
logging.getLogger().setLevel(logging.DEBUG)
class NoteLogger(logging.getLoggerClass()):
    def note(self, msg, *args, **kwargs):
        if self.isEnabledFor(logging.NOTE):
            self._log(logging.NOTE, msg, args, **kwargs)

logging.setLoggerClass(NoteLogger)
logging.note = partial(NoteLogger.note, logging.getLogger())