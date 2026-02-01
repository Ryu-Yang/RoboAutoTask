import re
import time

from playwright.sync_api import Playwright, sync_playwright, expect, Browser
import logging_mp
from dataclasses import dataclass

logger = logging_mp.get_logger(__name__)

@dataclass
class Operator:
    """平台网页自动操作器"""
    def __init__(self, args):
        self.playwright: Playwright = sync_playwright().start()
        self.headless = args.headless
        self.browser: Browser = self.playwright.chromium.launch(headless=self.headless)
        self.context = self.browser.new_context()

        self.url = args.url
        self.user = args.user
        self.password = args.password

        self.task = None
        self.task_id = args.task_id
        self.task_wait_timeout = args.task_wait_timeout
        
    def login(self):
        self.page = self.context.new_page()

        # 登录平台
        logger.info(f"正在访问: {self.url}")
        self.page.goto(self.url)
        
        logger.info(f"正在登录，用户名: {self.user}")
        self.page.get_by_role("textbox", name="请输入账号").click()
        self.page.get_by_role("textbox", name="请输入账号").fill(self.user)
        
        self.page.get_by_role("textbox", name="请输入登录密码").click()
        self.page.get_by_role("textbox", name="请输入登录密码").fill(self.password)
        
        self.page.get_by_role("button", name="登录").click()

        time.sleep(2)

    def find_task(self):
        # 查找指定任务ID的行并点击采集按钮
        try:
            logger.info(f"正在查找任务ID: {self.task_id}")
            # 等待表格加载
            self.page.wait_for_selector(".el-table__row", timeout=self.task_wait_timeout)
            
            # 找到所有行
            rows = self.page.locator(".el-table__row").all()
            target_row = None
            
            for row in rows:
                # 获取该行的第一列（ID列）的文本
                id_cell = row.locator(".el-table_1_column_1 .cell").first
                if id_cell.count() > 0:
                    cell_text = id_cell.inner_text().strip()
                    if cell_text == str(self.task_id):
                        target_row = row
                        break
            
            if target_row:
                logger.info(f"✓ 找到任务ID: {self.task_id}")
                # 在该行中查找"采集"按钮
                capture_button = target_row.locator("button.btn-capture")
                if capture_button.count() > 0:
                    capture_button.click()
                    logger.info(f"✓ 已点击任务 {self.task_id} 的采集按钮")
                    time.sleep(2)
                else:
                    logger.info(f"✗ 任务 {self.task_id} 未找到采集按钮（可能状态不可采集）")
            else:
                logger.info(f"✗ 未找到任务ID: {self.task_id}")
                
        except Exception as e:
            logger.error(f"✗ 查找任务时出错: {e}")
        
        # logger.info("等待10秒后关闭...")
        # time.sleep(10)

    # def exec_task(self):
    #     pass

    def start_task(self):
        """点击开始采集按钮"""
        try:
            logger.info("正在查找开始采集按钮...")
            
            # 等待页面加载，确保按钮出现
            self.page.wait_for_load_state("networkidle")
            
            start_button = self.page.locator('button.el-button--primary >> text=开始采集')
            
            if start_button.count() > 0:
                # 检查按钮是否可用（aria-disabled属性）
                is_disabled = start_button.get_attribute('aria-disabled')
                
                if is_disabled and is_disabled == 'true':
                    logger.warning("开始采集按钮当前不可用（disabled）")
                    
                    # 可选：检查按钮是否可见且可点击
                    if start_button.is_visible():
                        logger.info("按钮可见但被禁用，可能处于加载状态，等待3秒...")
                        time.sleep(3)
                        
                        # 重新检查状态
                        is_disabled = start_button.get_attribute('aria-disabled')
                        if is_disabled and is_disabled == 'false':
                            start_button.click()
                            logger.info("✓ 已点击开始采集按钮")
                        else:
                            logger.error("✗ 开始采集按钮仍然不可用")
                            return False
                    else:
                        logger.error("✗ 开始采集按钮不可见")
                        return False
                else:
                    # 按钮可用，直接点击
                    start_button.click()
                    logger.info("✓ 已点击开始采集按钮")
                    
                    # 等待操作生效
                    time.sleep(3)
                    
                    # 可选：验证是否开始成功（例如检查是否有确认提示）
                    try:
                        # 假设成功后会有一个提示消息
                        success_msg = self.page.locator('.el-message--success')
                        if success_msg.count() > 0:
                            logger.info("✓ 开始采集成功")
                    except:
                        # 没有成功提示也没关系，继续执行
                        pass
                        
                    return True
            else:
                logger.error("✗ 未找到开始采集按钮")
                
                # 调试：打印页面上所有的按钮
                all_buttons = self.page.locator('button').all()
                logger.info(f"页面上共有 {len(all_buttons)} 个按钮")
                for i, btn in enumerate(all_buttons[:10]):  # 只打印前10个
                    try:
                        text = btn.inner_text()[:50]  # 截取前50个字符
                        logger.info(f"按钮 {i}: {text}")
                    except:
                        pass
                        
                return False
                
        except Exception as e:
            logger.error(f"✗ 点击开始采集按钮时出错: {e}")
            
            # 尝试截图以便调试
            try:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                screenshot_path = f"screenshot_error_{timestamp}.png"
                self.page.screenshot(path=screenshot_path)
                logger.info(f"已保存错误截图到: {screenshot_path}")
            except:
                pass
                
            return False

    def complete_task(self):
        """点击 完成采集 按钮"""
        button_str: str = "完成采集"

        try:
            logger.info(f"正在查找 \"{button_str}\" 按钮...")

            self.page.wait_for_load_state("networkidle")
            
            button = self.page.locator(f'button.el-button--primary >> text={button_str}')
            button.wait_for(state="visible", timeout=5000)
            if button.count() > 0:
                button.click()
                logger.info(f"✓ 已点击 \"{button_str}\" 按钮")
                
                time.sleep(2)
                return True
            else:
                logger.error(f"✗ 未找到 \"{button_str}\" 按钮")
                return False

        except Exception as e:
            logger.error(f"✗ 点击 \"{button_str}\" 按钮时出错: {e}")            
            return False

    def commit_task(self):
        """点击 提交并继续 按钮"""
        button_str: str = "提交并继续"

        try:
            logger.info(f"正在查找 \"{button_str}\" 按钮...")

            self.page.wait_for_load_state("networkidle")

            button = self.page.locator(f'button.el-button--primary >> text={button_str}')
            button.wait_for(state="visible", timeout=5000)
            if button.count() > 0:
                button.click()
                logger.info(f"✓ 已点击 \"{button_str}\" 按钮")
                
                time.sleep(2)
                return True
            else:
                logger.error(f"✗ 未找到 \"{button_str}\" 按钮")
                return False

        except Exception as e:
            logger.error(f"✗ 点击 \"{button_str}\" 按钮时出错: {e}")            
            return False

    def quit_task(self):
        """点击退出采集按钮"""
        try:
            logger.info("正在查找退出采集按钮...")
            
            # 等待页面加载，确保按钮出现
            self.page.wait_for_load_state("networkidle")
            
            # 修改选择器为退出采集按钮
            quit_button = self.page.locator('button.el-button >> text=退出采集')
            
            # 等待按钮出现
            quit_button.wait_for(state="visible", timeout=5000)
            
            if quit_button.count() > 0:
                quit_button.click()
                logger.info("✓ 已点击退出采集按钮")
                
                # 等待操作生效
                time.sleep(2)
                
                # 验证退出是否成功
                try:
                    # 检查是否有成功提示消息
                    success_msg = self.page.locator('.el-message--success')
                    if success_msg.count() > 0:
                        logger.info("✓ 退出采集成功")
                    
                    # 或者检查按钮是否消失/变化
                    quit_button_after = self.page.locator('button.el-button--primary >> text=退出采集')
                    if quit_button_after.count() == 0:
                        logger.info("✓ 退出采集按钮已消失，退出成功")
                    else:
                        # 检查是否变成了其他按钮（如重新开始）
                        start_button = self.page.locator('button.el-button--primary >> text=开始采集')
                        if start_button.count() > 0:
                            logger.info("✓ 已成功退出采集，现在显示开始采集按钮")
                            
                except Exception as e:
                    logger.info(f"退出操作完成，但验证时出现: {e}")
                    
                return True
            else:
                logger.error("✗ 未找到退出采集按钮")
                
                # 调试：打印页面上所有的按钮
                all_buttons = self.page.locator('button').all()
                logger.info(f"页面上共有 {len(all_buttons)} 个按钮")
                for i, btn in enumerate(all_buttons[:10]):  # 只打印前10个
                    try:
                        text = btn.inner_text()[:50]  # 截取前50个字符
                        logger.info(f"按钮 {i}: {text}")
                    except:
                        pass
                        
                # 尝试其他可能的文本变体
                variant_texts = ["退出", "停止采集", "结束采集", "取消采集"]
                for text in variant_texts:
                    variant_button = self.page.locator(f'button:has-text("{text}")')
                    if variant_button.count() > 0:
                        logger.info(f"找到可能的退出按钮变体: {text}")
                        variant_button.click()
                        logger.info(f"✓ 已点击 {text} 按钮")
                        return True
                        
                return False
                
        except Exception as e:
            logger.error(f"✗ 点击退出采集按钮时出错: {e}")
            
            # 尝试截图以便调试
            try:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                screenshot_path = f"screenshot_quit_error_{timestamp}.png"
                self.page.screenshot(path=screenshot_path)
                logger.info(f"已保存错误截图到: {screenshot_path}")
            except:
                pass
                
            return False
    
    def stop(self) -> None:
        self.context.close()
        self.browser.close()
