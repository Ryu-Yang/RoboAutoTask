import datetime
import logging_mp
import numpy as np

from roboautotask.configs.svd import T_tool_tip
from roboautotask.utils.math import pose_to_matrix, matrix_to_pose


logger = logging_mp.get_logger(__name__)


class Calibrator():
    def __init__(self, csv_path: str):
        self.csv_path = csv_path
        self.is_running = True
        
        self._init_csv_file()
        
    def _init_csv_file(self) -> None:
        """初始化CSV文件并写入表头"""
        header = [
            'timestamp,arm_x,arm_y,arm_z,num_aruco,aruco_data',
            '# Format: ts,arm_x,arm_y,arm_z,n,id1,x1,y1,z1,id2,...'
        ]
        
        with open(self.csv_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(header) + '\n')
        logger.info(f'Calibration CSV initialized: {self.csv_path}')

    def _calculate_end_effector_position(self, pos, ori) -> np.ndarray:
        """计算机械臂末端执行器位置"""
        T = pose_to_matrix([pos[0], pos[1], pos[2]], 
                          [ori[0], ori[1], ori[2], ori[3]])
        
        T_base_tip = T @ T_tool_tip
        end_pos, _ = matrix_to_pose(T_base_tip)
        
        return end_pos

    def record_once(self, arm_pos, arm_ori, centers):
        """记录函数
        args:
            arm_pos: [x,y,z]
            arm_ori: [x,y,z,w]
            centers: {marker_id: (x, y, z)}
        """

        if arm_pos is None:
            logger.warning('No arm pose received yet')
            return False
        if centers is None:
            logger.warning('No marker center received yet')
            return False

        ts = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

        eepose = self._calculate_end_effector_position(arm_pos, arm_ori)

        arm_str = f'{eepose[0].item():.6f},{eepose[1].item():.6f},{eepose[2].item():.6f}'
        
        sorted_ids = sorted(centers.keys())
        aruco_str = f'{len(centers)}' + ''.join(f',{i},{centers[i][0]:.6f},{centers[i][1]:.6f},{centers[i][2]:.6f}'
                    for i in sorted_ids)
        line = f'{ts},{arm_str},{aruco_str}\n'

        with open(self.csv_path, 'a', encoding='utf-8') as f:
            f.write(line)
        logger.info(f'Recorded: arm ({eepose[0].item():.3f}, {eepose[1].item():.3f}, {eepose[2].item():.3f})  {len(centers)} marker(s)')

        return True
    
    def stop(self) -> None:
        """停止校准器"""
        if self.is_running:
            self.is_running = False
            logger.info('Calibrator stopped')
