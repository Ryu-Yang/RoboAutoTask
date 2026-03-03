import logging_mp
from roboautotask.robot.driver import MeasureCurrPose, execute_motion
from roboautotask.utils.pose import load_pose_from_file
from roboautotask.configs.robot import ROBOT_START_POS, ROBOT_START_ORI
from roboautotask.robot.daemon import Daemon

logging_mp.basic_config(level=logging_mp.INFO)
logger = logging_mp.get_logger(__name__)

def reset(daemon: Daemon):
    try:
        # MeasureCurrPose()
        # start_pos, start_quat = load_pose_from_file("latest_pose.txt")
        daemon.execute_motion(ROBOT_START_POS, ROBOT_START_ORI, 60, 100.0)
    except Exception:
        logger.exception("检测到异常详细信息:") 
        
        logger.error("--------------------------")
        logger.error("RESET FAILED")
        logger.error("--------------------------")
    finally:
        pass



    
if __name__ == "__main__":
    reset()
