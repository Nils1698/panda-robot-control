import sys
import os
import logging
from pathlib import Path
from main_GUI import MainWindow

base_folder = os.path.dirname(__file__)
tmp_folder = os.path.join(base_folder, 'tmp')
Path(tmp_folder).mkdir(exist_ok=True)

logging.basicConfig(filename=os.path.join(tmp_folder, 'errors.log'), level=logging.ERROR)
logger = logging.getLogger(__name__)
handler = logging.StreamHandler(stream=sys.stdout)
logger.addHandler(handler)

def handle_exception(exc_type, exc_value, exc_traceback):
    if issubclass(exc_type, KeyboardInterrupt):
        sys.__excepthook__(exc_type, exc_value, exc_traceback)
        return

    logger.error("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))

sys.excepthook = handle_exception

if __name__ == "__main__":
    
    try:
        app = MainWindow(db_filename=os.path.join(tmp_folder, 'GUIstate_GraphMain'), backup_filename=os.path.join(tmp_folder, 'sampler.txt'), appdata_dir=tmp_folder)
        app.mainloop()

    except Exception as e:
        print(e)
        logger.info(e)

