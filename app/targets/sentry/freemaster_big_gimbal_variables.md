# FreeMASTER 大云台调试变量

## 使用方法

这些变量定义在 `main.cc` 的全局符号区，具有 C 链接和 `volatile`
属性。FreeMASTER 可以直接从 ELF 符号表按下列名称搜索并添加到 Watch
或 Scope 中，不需要 UART 遥测帧。

主循环每 1 ms 更新一次变量。

## 模式和 PID 参数

| 变量名 | 类型 | 含义 |
|---|---|---|
| `fm_big_gimbal_force_mode` | `uint8_t` | 0=NoForce, 1=Init, 2=YesForce |
| `fm_big_gimbal_control_mode` | `uint8_t` | 0=NoForce, 1=Init, 2=RC, 3=Navi, 4=Keep |
| `fm_big_gimbal_rc_mode` | `uint8_t` | 0=遥控输入关闭, 1=遥控输入开启 |
| `fm_big_gimbal_chassis_mode` | `uint8_t` | 0=NoForce, 1=Init, 2=Follow, 3=Slow, 4=Quick, 5=Random, 6=Keep |
| `fm_big_yaw_angle_kp` | `float` | 角度环 Kp |
| `fm_big_yaw_angle_ki` | `float` | 角度环 Ki |
| `fm_big_yaw_angle_kd` | `float` | 角度环 Kd |
| `fm_big_yaw_speed_kp` | `float` | 速度环 Kp |
| `fm_big_yaw_speed_ki` | `float` | 速度环 Ki |
| `fm_big_yaw_speed_kd` | `float` | 速度环 Kd |

## 角度环目标、反馈和输出

| 变量名 | 单位 | 含义 |
|---|---|---|
| `fm_big_yaw_target_deg` | deg | 目标 yaw |
| `fm_big_yaw_current_deg` | deg | AHRS 当前 yaw |
| `fm_big_yaw_error_deg` | deg | 角度误差 |
| `fm_big_yaw_rate_target_dps` | deg/s | 角度环输出，即速度环目标 |
| `fm_big_yaw_rate_current_dps` | deg/s | BMI088 当前 yaw 角速度 |
| `fm_big_yaw_rate_error_dps` | deg/s | 速度误差 |
| `fm_big_yaw_angle_p_out` | deg/s | 角度环 P 输出 |
| `fm_big_yaw_angle_i_out` | deg/s | 角度环 I 输出 |
| `fm_big_yaw_angle_d_out` | deg/s | 角度环 D 输出 |

## 速度环输出和电机反馈

| 变量名 | 单位 | 含义 |
|---|---|---|
| `fm_big_yaw_speed_p_out` | 原始值 | 速度环 P 输出 |
| `fm_big_yaw_speed_i_out` | 原始值 | 速度环 I 输出 |
| `fm_big_yaw_speed_d_out` | 原始值 | 速度环 D 输出 |
| `fm_big_yaw_speed_output` | 原始值 | 速度环总输出 |
| `fm_big_yaw_torque_nm` | N·m | 送入达妙电机的前馈力矩 |
| `fm_big_yaw_motor_deg` | deg | 下 yaw 电机实测角度 |
| `fm_big_yaw_motor_rate_dps` | deg/s | 下 yaw 电机实测角速度 |
| `fm_big_rc_yaw_input` | -1～1 | 遥控器左摇杆 X 输入 |

## 遥控模式检查

遥控模式下应看到：

- `fm_big_gimbal_force_mode == 2`
- `fm_big_gimbal_control_mode == 2`
- `fm_big_gimbal_rc_mode == 1`
- `fm_big_rc_yaw_input` 随左摇杆 X 变化
- `fm_big_yaw_target_deg` 随遥控输入变化
- `fm_big_yaw_current_deg` 跟随目标
- `fm_big_yaw_speed_output` 应在 `-50000～50000` 内工作
