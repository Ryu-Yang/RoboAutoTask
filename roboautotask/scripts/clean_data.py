#!/usr/bin/env python3
"""
删除脚本 - 根据 meta/common_record.json 中的 last_upload_id 是否为 1 删除对应任务目录。
目录结构：/home/xxx/DoRobot/dataset/<日期>/<user|dev>/<任务名>/<任务名_id>/meta/common_record.json
如果 last_upload_id == 1，则删除整个 <任务名_id> 目录，并递归删除空父目录。
支持参数：--days 指定删除几天前的数据（默认3），--root 指定根目录，--dry-run 仅显示操作。
"""

import os
import sys
import json
import shutil
import argparse
from datetime import datetime, timedelta


def get_threshold_date(days):
    """计算阈值日期（YYYYMMDD），返回字符串"""
    today = datetime.now().date()
    threshold = today - timedelta(days=days)
    return threshold.strftime("%Y%m%d")


def should_delete(json_path):
    """检查指定 JSON 文件中的 last_upload_id 是否为 1"""
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return data.get('last_upload_id') == 1
    except (FileNotFoundError, json.JSONDecodeError, KeyError, IOError) as e:
        # 文件不存在、JSON 无效或缺少字段时视为不删除
        print(f"  无法读取或解析 {json_path}，错误：{e}")
        return False


def delete_dir(path, dry_run):
    """删除目录（如果存在），dry_run 为 True 时仅打印"""
    if not os.path.exists(path):
        return
    if dry_run:
        print(f"[DRY RUN] 删除目录: {path}")
    else:
        try:
            shutil.rmtree(path)
            print(f"已删除: {path}")
        except Exception as e:
            print(f"删除失败 {path}: {e}")


def cleanup_empty_parents(start_path, root, dry_run):
    """
    从 start_path 的父目录开始向上检查空目录并删除，直到根目录（不含根）。
    start_path: 已删除的任务ID目录的绝对路径
    root: 根目录绝对路径，不删除根及其之上的目录
    """
    current = os.path.dirname(start_path)  # 任务名目录
    while current.startswith(root) and current != root:
        try:
            if os.path.exists(current) and not os.listdir(current):
                if dry_run:
                    print(f"[DRY RUN] 删除空目录: {current}")
                else:
                    os.rmdir(current)
                    print(f"已删除空目录: {current}")
                current = os.path.dirname(current)  # 继续向上
            else:
                break  # 非空则停止向上
        except OSError as e:
            print(f"检查/删除目录 {current} 时出错: {e}")
            break


def process_date_dir(date_dir_path, date_str, threshold_str, root, dry_run):
    """处理单个日期目录，如果日期 <= threshold_str 则处理"""
    # 日期比较：目录名应为8位数字
    if not date_str.isdigit() or len(date_str) != 8:
        return
    if int(date_str) > int(threshold_str):
        return  # 大于阈值，不处理

    print(f"处理日期目录: {date_dir_path}")

    # 遍历 user 和 dev 子目录
    for type_dir in ['user', 'dev']:
        type_path = os.path.join(date_dir_path, type_dir)
        if not os.path.isdir(type_path):
            continue

        # 遍历任务名目录
        for task_name in os.listdir(type_path):
            task_path = os.path.join(type_path, task_name)
            if not os.path.isdir(task_path):
                continue

            # 遍历任务ID目录
            for task_id in os.listdir(task_path):
                task_id_path = os.path.join(task_path, task_id)
                if not os.path.isdir(task_id_path):
                    continue

                meta_path = os.path.join(task_id_path, 'meta', 'common_record.json')
                # print(f"read file: {meta_path}")
                if not os.path.isfile(meta_path):
                    continue

                if should_delete(meta_path):
                    # 删除任务ID目录
                    delete_dir(task_id_path, dry_run)
                    # 向上清理空目录（任务名目录、type目录、日期目录）
                    cleanup_empty_parents(task_id_path, root, dry_run)


def main():
    parser = argparse.ArgumentParser(description="根据 last_upload_id 删除历史数据目录")
    parser.add_argument('--days', type=int, default=3,
                        help='删除几天前的数据（默认3）')
    parser.add_argument('--root', type=str,
                        default='/home/xxx/DoRobot/dataset/',
                        help='根目录路径（默认 /home/xxx/DoRobot/)')
    parser.add_argument('--dry-run', action='store_true',
                        help='仅显示要删除的目录，不实际执行删除')
    args = parser.parse_args()

    root = os.path.abspath(args.root + "dataset/")
    if not os.path.isdir(root):
        print(f"错误：根目录不存在或无法访问: {root}")
        sys.exit(1)

    days = max(0, args.days)  # 确保天数非负
    threshold_str = get_threshold_date(days)

    print(f"当前时间: {datetime.now()}")
    print(f"阈值日期: {threshold_str} (今天往前 {days} 天)")
    print(f"根目录: {root}")
    if args.dry_run:
        print("*** 模拟运行模式，不会实际删除任何文件 ***")

    # 遍历根目录下的日期子目录
    for item in os.listdir(root):
        date_dir_path = os.path.join(root, item)

        # print(f"处理日期目录: {date_dir_path}")

        if os.path.isdir(date_dir_path):
            # print(f"处理日期目录: {date_dir_path}")
            process_date_dir(date_dir_path, item, threshold_str, root, args.dry_run)

    print("处理完成。")


if __name__ == '__main__':
    main()