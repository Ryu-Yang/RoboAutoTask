import re
from playwright.sync_api import Playwright, sync_playwright, expect
import time

def run(playwright: Playwright) -> None:
    browser = playwright.chromium.launch(headless=False)
    context = browser.new_context()
    page = context.new_page()
    page.goto("https://cn.bing.com/")
    page.get_by_role("searchbox", name="Enter your search term").click()
    page.get_by_role("searchbox", name="Enter your search term").fill("lerobot")
    page.get_by_role("searchbox", name="Enter your search term").press("Enter")
    with page.expect_popup() as page1_info:
        page.get_by_role("link", name="GitHub - huggingface/lerobot").click()
    page1 = page1_info.value

    time.sleep(5)

    # ---------------------
    context.close()
    browser.close()


with sync_playwright() as playwright:
    run(playwright)
