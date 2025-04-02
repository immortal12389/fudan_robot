#!/usr/bin/python
# -*- coding:utf-8 -*-
import requests
import json
import sys
import time

def send_post_request(url, headers, json_data):
    try:
        response = requests.post(url, headers=headers, json=json_data, timeout=30)
        response.raise_for_status()
        return response
    except requests.exceptions.RequestException as e:
        print(f"POST 请求失败: {e}")
        sys.exit(1)

def send_get_request(url, headers):
    try:
        response = requests.get(url, headers=headers, timeout=30)
        response.raise_for_status()
        return response
    except requests.exceptions.RequestException as e:
        print(f"GET 请求失败: {e}")
        return None

def main(ip):
    headers = {'Content-Type': 'application/json'}
    url1 = f"http://{ip}:1448/api/core/motion/v1/actions"
    url2 = f"http://{ip}:1448/api/core/motion/v1/actions/{{action_id}}"
    url3 = f"http://{ip}:1448/api/core/artifact/v1/lines/tracks"

    action_options1 = {
        "action_name": "slamtec.agent.actions.MoveToAction",
        "options": {
            "target": {
                "x": 0.42,
                "y": 0.81,
                "z": 0
            },
            "move_options": {
                "mode": 0,
                "flags": [],
                "yaw": 0,
                "acceptable_precision": 0,
                "fail_retry_count": 0
            }
        }
    }

    action_options2 = {
        "action_name": "slamtec.agent.actions.MoveToAction",
        "options": {
            "target": {
                "x": 0.37,
                "y": 2.1,
                "z": 0
            },
            "move_options": {
                "mode": 1,
                "flags": [],
                "yaw": 0,
                "acceptable_precision": 0,
                "fail_retry_count": 0
            }
        }
    }

    data = [
        {
            "id": 101,
            "start": {
                "x": 0.4,
                "y": 0.8
            },
            "end": {
                "x": 0.37,
                "y": 2.1
            }
        }
    ]

    # 发送移动指令
    print("机器人开始移动到点")
    rq1 = send_post_request(url1, headers, action_options1)
    print(f"接口调用成功: {rq1.text}\n")

    # 获取 action_id
    actionid1 = json.loads(rq1.text).get("action_id")
    if not actionid1:
        print("无法获取 action_id")
        sys.exit(1)
    print(f"action_id 为: {actionid1}")

    # 等待任务结束
    print("************等待到点任务1结束************")
    action_url = url2.format(action_id=actionid1)

    while True:
        rq2 = send_get_request(action_url, headers)
        if rq2 is None:
            continue

        print(f"查询 action 状态为: {rq2.text}")
        if not json.loads(rq2.text).get("action_name"):
            print("任务1结束")
            break

        time.sleep(2)

    # 添加虚拟轨道
    rq3 = send_post_request(url3, headers, data)
    print(f"接口调用成功,: {rq3.text}\n")

    # 严格轨道到点
    rq4 = send_post_request(url1, headers, action_options2)
    actionid2 = json.loads(rq4.text).get("action_id")
    if not actionid2:
        print("无法获取 action_id")
        sys.exit(1)
    print(f"action_id 为: {actionid2}")

    print("************等待到点任务2结束************")
    action_url = url2.format(action_id=actionid2)

    while True:
        rq5 = send_get_request(action_url, headers)
        if rq5 is None:
            continue

        print(f"查询 action 状态为: {rq5.text}")
        if not json.loads(rq5.text).get("action_name"):
            print("任务2结束")
            break

        time.sleep(2)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("请提供IP地址")
        sys.exit(1)

    main(sys.argv[1])









