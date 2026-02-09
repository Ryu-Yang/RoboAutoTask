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
        """查找指定任务ID的任务"""
        try:
            logger.info(f"正在查找任务ID: {self.task_id}")
            
            # 切换到任务ID搜索
            try:
                # 点击下拉框
                dropdown = self.page.locator(".field-selector .el-select__wrapper")
                dropdown.click()
                time.sleep(0.5)
                
                # 选择"任务ID"选项
                task_id_option = self.page.locator(".el-select-dropdown__item:has-text('任务ID')")
                if task_id_option.count() > 0:
                    task_id_option.click()
                    logger.info("✓ 已切换到任务ID搜索")
                    time.sleep(0.5)
            except:
                logger.info("下拉框切换失败，继续使用当前搜索字段")
            
            # 输入任务ID并搜索
            try:
                search_input = self.page.locator(".search-input .el-input__inner")
                search_input.fill(str(self.task_id))
                time.sleep(0.5)
                
                search_button = self.page.locator(".search-button")
                search_button.click()
                logger.info(f"✓ 已搜索任务ID: {self.task_id}")
                time.sleep(2)
            except:
                logger.info("搜索失败，直接查找表格")
            
        except Exception as e:
            logger.error(f"✗ 查找任务时出错: {e}")

    def exec_task(self):
        """进入指定任务ID的任务"""
        try:
            logger.info(f"尝试进入任务ID: {self.task_id}")

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
            logger.error(f"✗ 进入任务时出错: {e}")
        
    def click_button(self, button_str: str, primary: bool = True):
        """按钮点击函数"""
        try:
            logger.info(f"正在查找 \"{button_str}\" 按钮...")

            self.page.wait_for_load_state("networkidle")

            if primary:
                button = self.page.locator(f'button.el-button--primary >> text={button_str}')
            else:
                button = self.page.locator(f'button.el-button >> text={button_str}')

            button.wait_for(state="visible", timeout=5000)
            if button.count() > 0:

                button.click()
                logger.info(f"✓ 已点击 \"{button_str}\" 按钮")
                
                time.sleep(2)
                return True
            else:
                logger.error(f"✗ 未找到 \"{button_str}\" 按钮")

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
            logger.error(f"✗ 点击 \"{button_str}\" 按钮时出错: {e}")            
            return False

    def start_task(self):
        """点击 开始采集 按钮"""
        self.click_button("开始采集")

    def complete_task(self):
        """点击 完成采集 按钮"""
        self.click_button("完成采集")

    def commit_task(self):
        """点击 提交并继续 按钮"""
        self.click_button("提交并继续")

    def destroy_task(self):
        """点击 丢弃重采 按钮"""
        self.click_button("丢弃重采", primary=False)

    def quit_task(self):
        """点击 退出采集 按钮"""
        self.click_button("退出采集", primary=False)
    
    def stop(self) -> None:
        self.context.close()
        self.browser.close()
