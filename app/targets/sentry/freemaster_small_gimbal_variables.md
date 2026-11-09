# FreeMASTER 小云台调试变量

这些变量定义在 `main.cc` 的全局符号区，具有 C 链接和 `volatile`
属性。FreeMASTER 可以直接从 ELF 符号表按变量名搜索并添加到 Watch
或 Scope 中。主循环每 1 ms 更新一次。

## 模式和状态

| 变量名 | 含义 |
|---|---|
| `fm_small_gimbal_force_mode` | 0=NoForce, 1=Init, 2=YesForce |
| `fm_small_gimbal_basic_mode` | 0=Match, 1=Vision, 2=Ec |
| `fm_small_gimbal_control_mode` | 小云台控制模式编号 |
| `fm_small_gimbal_rc_mode` | 0=遥控输入关闭, 1=遥控输入开启 |
| `fm_small_gimbal_ammo_mode` | 0=NoForce, 1=NoSpeed, 2=NormalSpeed |
| `fm_small_gimbal_rotor_mode` | 0=NoForce, 1=Stop, 2=Forward, 3=Single |

## yaw PID 参数

| 变量名 | 含义 |
|---|---|
| `fm_small_yaw_angle_kp` | yaw 角度环 Kp |
| `fm_small_yaw_angle_ki` | yaw 角度环 Ki |
| `fm_small_yaw_angle_kd` | yaw 角度环 Kd |
| `fm_small_yaw_speed_kp` | yaw 速度环 Kp |
| `fm_small_yaw_speed_ki` | yaw 速度环 Ki |
| `fm_small_yaw_speed_kd` | yaw 速度环 Kd |

## yaw 目标、反馈和输出

| 变量名 | 单位 | 含义 |
|---|---|---|
| `fm_small_yaw_target_deg` | deg | 目标 yaw |
| `fm_small_yaw_current_deg` | deg | Hipnuc 当前 yaw |
| `fm_small_yaw_angle_feedback_deg` | deg | 角度环实际反馈 |
| `fm_small_yaw_motor_deg` | deg | 上 yaw 电机位置 |
| `fm_small_yaw_error_deg` | deg | 角度误差 |
| `fm_small_yaw_rate_target_dps` | deg/s | 速度环目标 |
| `fm_small_yaw_rate_current_dps` | deg/s | Hipnuc 原始角速度 |
| `fm_small_yaw_rate_feedback_dps` | deg/s | 速度环实际反馈 |
| `fm_small_yaw_rate_error_dps` | deg/s | 速度误差 |
| `fm_small_yaw_angle_p_out` | deg/s | 角度环 P 输出 |
| `fm_small_yaw_angle_i_out` | deg/s | 角度环 I 输出 |
| `fm_small_yaw_angle_d_out` | deg/s | 角度环 D 输出 |
| `fm_small_yaw_speed_p_out` | 原始值 | 速度环 P 输出 |
| `fm_small_yaw_speed_i_out` | 原始值 | 速度环 I 输出 |
| `fm_small_yaw_speed_d_out` | 原始值 | 速度环 D 输出 |
| `fm_small_yaw_speed_output` | 原始值 | 速度环总输出 |
| `fm_small_yaw_motor_output` | 原始值 | 最终发送给 GM6020 的输出 |
| `fm_small_yaw_motor_rate_dps` | deg/s | GM6020 实测角速度 |

## pitch PID 参数

| 变量名 | 含义 |
|---|---|
| `fm_small_pitch_angle_kp` | pitch 角度环 Kp |
| `fm_small_pitch_angle_ki` | pitch 角度环 Ki |
| `fm_small_pitch_angle_kd` | pitch 角度环 Kd |
| `fm_small_pitch_speed_kp` | pitch 速度环 Kp |
| `fm_small_pitch_speed_ki` | pitch 速度环 Ki |
| `fm_small_pitch_speed_kd` | pitch 速度环 Kd |

## pitch 目标、反馈和输出

| 变量名 | 单位 | 含义 |
|---|---|---|
| `fm_small_pitch_target_deg` | deg | 目标 pitch |
| `fm_small_pitch_current_deg` | deg | Hipnuc 当前 pitch |
| `fm_small_pitch_angle_feedback_deg` | deg | 角度环实际反馈 |
| `fm_small_pitch_motor_deg` | deg | 达妙 pitch 电机位置 |
| `fm_small_pitch_error_deg` | deg | 角度误差 |
| `fm_small_pitch_rate_target_dps` | deg/s | 速度环目标 |
| `fm_small_pitch_rate_current_dps` | deg/s | Hipnuc 原始角速度 |
| `fm_small_pitch_rate_feedback_dps` | deg/s | 速度环实际反馈 |
| `fm_small_pitch_rate_error_dps` | deg/s | 速度误差 |
| `fm_small_pitch_angle_p_out` | deg/s | 角度环 P 输出 |
| `fm_small_pitch_angle_i_out` | deg/s | 角度环 I 输出 |
| `fm_small_pitch_angle_d_out` | deg/s | 角度环 D 输出 |
| `fm_small_pitch_speed_p_out` | 原始值 | 速度环 P 输出 |
| `fm_small_pitch_speed_i_out` | 原始值 | 速度环 I 输出 |
| `fm_small_pitch_speed_d_out` | 原始值 | 速度环 D 输出 |
| `fm_small_pitch_speed_output` | 原始值 | 速度环总输出 |
| `fm_small_pitch_motor_output` | 原始值 | 含重力补偿的最终 pitch 输出 |
| `fm_small_pitch_motor_rate_dps` | deg/s | 达妙 pitch 电机实测角速度 |

## 遥控输入

| 变量名 | 含义 |
|---|---|
| `fm_small_rc_left_x` | 左摇杆 X，范围 -1～1 |
| `fm_small_rc_left_y` | 左摇杆 Y，范围 -1～1 |
