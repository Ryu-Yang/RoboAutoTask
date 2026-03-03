import re
from playwright.sync_api import Playwright, sync_playwright, expect


def run(playwright: Playwright) -> None:
    browser = playwright.chromium.launch(headless=False)
    context = browser.new_context()
    page = context.new_page()
    page.goto("https://www.baidu.com/")
    page.get_by_role("textbox", name="伊朗:美国无法掌控战争结局").click()
    page.get_by_role("textbox", name="伊朗:美国无法掌控战争结局").fill("lerobot")

    # ---------------------
    context.close()
    browser.close()


with sync_playwright() as playwright:
    run(playwright)
