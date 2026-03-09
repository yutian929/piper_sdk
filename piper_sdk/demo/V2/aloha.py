#!/usr/bin/env python3
# -*-coding:utf8-*-
# ALOHA 主从臂示教-复现
# 主臂(master)示教，从臂(slave)实时跟随复制主臂的关节角度和夹爪状态
# 需要两个独立的CAN接口分别连接主臂和从臂
#
# 用法:
#   python aloha.py --master can0 --slave can1
#   python aloha.py --master can0 --slave can1 --freq 200
#
# 注意:
#   1. 需要先 pip install piper_sdk
#   2. 两个机械臂分别连接不同的CAN接口
#   3. 先上电从臂，再上电主臂
#   4. 主臂设为主臂模式后，手动拖动主臂，从臂会实时跟随

import argparse
import signal
import sys
import time
import threading
from piper_sdk import *


class AlohaController:
    """主臂示教，从臂完全复制跟随"""

    def __init__(self, master_can: str, slave_can: str, freq: float = 200):
        self.master_can = master_can
        self.slave_can = slave_can
        self.period = 1.0 / freq
        self._running = False

        self.piper_master = None
        self.piper_slave = None

    def _init_master(self):
        """初始化主臂: 连接并设置为主臂模式"""
        print(f"[主臂] 连接 {self.master_can} ...")
        self.piper_master = C_PiperInterface_V2(self.master_can)
        self.piper_master.ConnectPort()
        time.sleep(0.1)
        # 设置为主臂模式(0xFA)
        self.piper_master.MasterSlaveConfig(0xFA, 0, 0, 0)
        time.sleep(0.5)
        print(f"[主臂] 已设置为主臂模式(示教臂)")

    def _init_slave(self):
        """初始化从臂: 连接、设置高跟随模式、使能"""
        print(f"[从臂] 连接 {self.slave_can} ...")
        self.piper_slave = C_PiperInterface_V2(self.slave_can)
        self.piper_slave.ConnectPort()
        time.sleep(0.1)

        # 设置从臂模式(0xFC)
        self.piper_slave.MasterSlaveConfig(0xFC, 0, 0, 0)
        time.sleep(0.5)

        # 设置CAN指令控制模式 + 关节控制 + 高跟随模式(0xAD)
        self.piper_slave.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
        time.sleep(0.1)

        # 使能从臂
        print("[从臂] 使能中...")
        retry = 0
        while not self.piper_slave.EnablePiper():
            retry += 1
            if retry > 500:
                print("[从臂] 使能超时，请检查硬件连接")
                sys.exit(1)
            # 持续发送高跟随模式指令
            self.piper_slave.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
            time.sleep(0.01)
        # 使能夹爪
        self.piper_slave.GripperCtrl(0, 1000, 0x01, 0)
        print("[从臂] 使能成功，高跟随模式已开启")

    def start(self):
        """启动ALOHA主从控制"""
        self._init_master()
        self._init_slave()

        self._running = True
        print("\n===== ALOHA 主从臂示教开始 =====")
        print("拖动主臂，从臂将实时跟随")
        print("按 Ctrl+C 停止\n")

        try:
            while self._running:
                t0 = time.monotonic()

                # 读取主臂关节控制指令
                master_joint = self.piper_master.GetArmJointCtrl()
                jc = master_joint.joint_ctrl

                # 读取主臂夹爪控制指令
                master_gripper = self.piper_master.GetArmGripperCtrl()
                gc = master_gripper.gripper_ctrl

                # 发送到从臂 - 关节
                self.piper_slave.JointCtrl(
                    jc.joint_1,
                    jc.joint_2,
                    jc.joint_3,
                    jc.joint_4,
                    jc.joint_5,
                    jc.joint_6,
                )

                # 发送到从臂 - 夹爪
                self.piper_slave.GripperCtrl(
                    abs(gc.grippers_angle),
                    gc.grippers_effort,
                    gc.status_code,
                    gc.set_zero,
                )

                # 保持控制频率
                elapsed = time.monotonic() - t0
                sleep_time = self.period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        """停止并安全关闭"""
        self._running = False
        print("\n[ALOHA] 正在停止...")
        if self.piper_slave is not None:
            try:
                self.piper_slave.DisableArm(7)
            except Exception:
                pass
        print("[ALOHA] 已停止")


def main():
    parser = argparse.ArgumentParser(
        description="ALOHA 主从臂示教: 主臂拖动示教，从臂实时完全复制"
    )
    parser.add_argument(
        "--master", type=str, default="m_r", help="主臂CAN接口名 (默认: can0)"
    )
    parser.add_argument(
        "--slave", type=str, default="s_r", help="从臂CAN接口名 (默认: can1)"
    )
    parser.add_argument(
        "--freq", type=float, default=200, help="控制频率Hz (默认: 200)"
    )
    args = parser.parse_args()

    controller = AlohaController(args.master, args.slave, args.freq)
    controller.start()


if __name__ == "__main__":
    main()
