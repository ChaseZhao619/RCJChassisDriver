# 贡献指南

感谢你参与 RCJChassisDriver。这个项目同时包含 STM32 固件、Qt 上位机、串口协议和硬件调参文档，因此贡献时需要同时关注软件质量和实车安全。

## 贡献流程

1. Fork 仓库或从当前仓库创建功能分支。
2. 在清晰命名的分支上修改，例如 `docs/update-protocol-guide`、`fix/chassis-stop-condition`。
3. 按影响范围更新代码、文档和测试记录。
4. 提交 pull request，并填写 PR 模板中的安全检查和测试结果。

## 提交前检查

通用检查：

```bash
git diff --check
```

建议同时检查 README/Markdown 内部链接是否有效。修改 GitHub issue 模板时，应确认 YAML 语法有效。

## 固件与协议改动要求

如果改动 `firmware/stm32_chassis` 中的代码，PR 中应说明：

- 改动的模块：`App`、`Bsp`、`Core`、`Drivers` 或构建配置。
- 是否改变串口命令 payload、单位、范围、默认值或回复格式。
- 是否影响 `ok`、`busy`、`done` 的发送时机。
- 是否需要同步更新上位机、树莓派端或 README。
- 是否在架空、低速、落地或比赛场地下测试。

协议变更应优先保持向后兼容。如果必须破坏兼容性，需要在文档中明确旧行为和新行为。

## BSP、硬件和调参改动要求

如果改动电机方向、PID、阈值、电流限制、PWM 脉宽、测试宏或外设引脚，PR 中必须记录：

- 测试硬件：电机、电调、传感器、供电电压、机械结构。
- 旧值、新值和调参依据。
- 测试条件：架空、低速、落地、场地、负载。
- 观察现象：超调、抖动、温升、堵转、漂移、响应延迟等。
- 安全措施：急停、限速、限流、隔离负载。

方向错误不能通过负 PID 参数修正，应优先修改方向宏并说明验证方法。

## 上位机改动要求

如果改动 `apps/desktop_controller`，PR 中应说明：

- 是否依赖新的固件协议。
- 地图坐标、单位、路径分段或避障策略是否变化。
- JSON 配置是否兼容旧版本。
- 是否新增按钮、快捷操作或自动发送命令的入口。
- 是否测试过无串口、串口断开、CRC 错误和固件返回 `busy`。

## 构建与环境说明

常用验证命令：

```bash
cmake --fresh --preset stm32-debug
cmake --build --preset stm32-debug
cmake --fresh --preset host-debug
cmake --build --preset host-debug
```

如果本机缺少依赖导致无法完整构建，应在 PR 中明确写出原因。例如：

- ARM 工具链缺 Newlib/标准头文件，报 `stdint.h`、`sys/stat.h`、`errno.h` 缺失。
- 本机未安装 Qt6 开发包，host app 配置失败。

## 提交信息建议

建议使用简洁的英文前缀：

- `docs:` 文档
- `fix:` bug 修复
- `feat:` 新功能
- `refactor:` 重构
- `build:` 构建配置
- `test:` 测试或测试工具

示例：

```text
docs: add hardware safety issue templates
refactor: standardize project directory structure
fix: reject invalid chassis command arguments
```
