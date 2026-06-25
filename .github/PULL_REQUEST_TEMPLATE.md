# Pull Request

## 改动摘要

请用 2-5 句话说明本 PR 做了什么，以及为什么需要这个改动。

## 改动类型

- [ ] firmware
- [ ] desktop app
- [ ] serial protocol
- [ ] BSP / hardware tuning
- [ ] documentation
- [ ] build / config
- [ ] repository maintenance

## 安全确认

- [ ] 本 PR 不会驱动真实硬件，或已说明硬件测试安全措施。
- [ ] 如果涉及底盘、执行器、PWM、继电器或电机输出，已说明是否架空、低速、限流或隔离负载。
- [ ] 如果修改测试宏，默认值保持关闭或已说明原因。
- [ ] 如果修改协议参数校验，非法参数、坏 CRC、超长帧和缺失参数不会触发危险动作。
- [ ] 如果修改 Flash 写入或 MCU 复位逻辑，已说明触发条件和防误触措施。

## 兼容性

- [ ] 不改变现有串口协议。
- [ ] 改变了串口协议，并已同步更新固件、上位机和 README。
- [ ] 不改变配置文件格式。
- [ ] 改变了配置文件格式，并已说明兼容或迁移方式。

## 测试记录

请勾选已执行项；未执行项请在下方说明原因。

- [ ] `git diff --check`
- [ ] README/Markdown 链接检查
- [ ] GitHub issue template YAML 检查
- [ ] `cmake --fresh --preset stm32-debug`
- [ ] `cmake --build --preset stm32-debug`
- [ ] `cmake --fresh --preset host-debug`
- [ ] `cmake --build --preset host-debug`
- [ ] 实车架空测试
- [ ] 实车低速落地测试

未执行或失败的测试原因：

```text

```

## 硬件/环境信息

如果涉及固件、BSP、运动控制或上位机串口，请填写：

```text
OS:
Branch/commit:
ARM toolchain:
Qt version:
MCU board:
Motor/ESC:
Sensors:
Power:
Test condition:
```

## 文档更新

- [ ] 不需要更新文档。
- [ ] 已更新 README 或 docs。
- [ ] 需要后续补文档，原因已说明。
