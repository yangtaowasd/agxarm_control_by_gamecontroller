# 硬件连接 / Hardware Connection

## 目标 / Goal

这些文件集中管理 Nero/Piper-L 的 SDK/CAN 发现和正式连接生命周期，不包含控制
算法。 / These files centralize SDK/CAN discovery and formal connection
lifecycle for Nero/Piper-L; they contain no control algorithm.

## 文件 / Files

| 文件 / File | 分类责任 / Classified responsibility |
| --- | --- |
| `armbycontroller/hardware/connection.py` | 用 `default` profile 探测、保存完整 firmware 数据、断开、映射版本并以新实例正式重连 / probe with the `default` profile, save complete firmware data, disconnect, map the version, and formally reconnect with a new instance |
| `armbycontroller/hardware/feedback.py` | 无副作用的 SDK 关节反馈归一化、完整性提取和低通差分速度估计 / side-effect-free SDK joint-feedback normalization, completeness extraction, and filtered velocity estimation |
| `armbycontroller/hardware/__init__.py` | 两阶段连接和反馈工具的稳定 Python 导出面 / stable Python exports for two-stage connection and feedback tools |
| `test/test_hardware_connection.py` | 两次连接的严格顺序、实例隔离、版本边界、数据副本和失败清理 / exact two-connection order, instance isolation, version boundaries, copied data, and failure cleanup |

## 生命周期 / Lifecycle

```text
create(DEFAULT) -> connect -> temporary enable -> get_firmware
       -> save -> disable -> disconnect -> wait 0.5 s
       -> create(detected profile) -> connect -> control
```

Nero 和 Piper-L 探测实例都发送一次临时 `enable()`，读取固件后在断开前发送
`disable()`；它们不发送模式切换、运动或固件写入命令。若没有
`software_version`、版本无法解析，或当前 pyAgxArm 不支持映射出的 profile，
正式实例不会建立。 / Both Nero and Piper-L probes send one temporary
`enable()` and send `disable()` before disconnecting after the firmware read.
They send no mode, motion, or firmware-write command. The formal instance is
not created when `software_version` is absent, cannot be parsed, or maps to a
profile unsupported by the installed pyAgxArm.

## 参数 / Parameters

- `firmware_probe_timeout`：探测总时限，默认 `5.0 s`。 / total discovery
  deadline, default `5.0 s`.
- `firmware_probe_poll_period`：空响应后的重试间隔，默认 `0.1 s`。 / retry
  interval after an empty response, default `0.1 s`.
- `firmware_reconnect_delay`：第一次断开后、创建第二个实例前的等待，默认
  `0.5 s`。 / delay after the first disconnect and before creating the second
  instance, default `0.5 s`.
- `firmware`：实机上检测结果优先；显式值用于提示不一致，并用于不连接 CAN 的
  dry-run profile。 / detected hardware wins on real arms; an explicit value
  reports mismatches and supplies the profile for CAN-free dry runs.
