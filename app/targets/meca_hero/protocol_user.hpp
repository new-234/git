/*
  Copyright (c) 2026 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/**
 * @file  librm/device/referee/protocol_user.hpp
 * @brief 裁判系统串口协议0x301用户自定义子协议(2026-05-14)
 */

#ifndef LIBRM_DEVICE_REFEREE_PROTOCOL_USER_HPP
#define LIBRM_DEVICE_REFEREE_PROTOCOL_USER_HPP

#include <librm.hpp>

#include <cstddef>
#include <typeindex>

#include <mapbox/eternal.hpp>

namespace rm::device {
#pragma pack(push, 1)
struct AllyRobotPosition {
  f32 hero_1_x;
  f32 hero_1_y;
  f32 engineer_2_x;
  f32 engineer_2_y;
  f32 standard_3_x;
  f32 standard_3_y;
  f32 standard_4_x;
  f32 standard_4_y;
  f32 reserved_1;
  f32 reserved_2;
};

struct EnemyRobotPosition {
  f32 hero_1_x;
  f32 hero_1_y;
  f32 engineer_2_x;
  f32 engineer_2_y;
  f32 standard_3_x;
  f32 standard_3_y;
  f32 standard_4_x;
  f32 standard_4_y;
  f32 drone_6_x;
  f32 drone_6_y;
  f32 sentry_7_x;
  f32 sentry_7_y;
};

struct EnemyRobotHP {
  u16 hero_1_HP;
  u16 engineer_2_HP;
  u16 standard_3_HP;
  u16 standard_4_HP;
  u16 reserved;
  u16 sentry_7_HP;
};

struct EnemyRobotProjectileAllowance {
  u16 hero_1_projectile_allowance;
  u16 standard_3_projectile_allowance;
  u16 standard_4_projectile_allowance;
  u16 drone_6_projectile_allowance;
  u16 sentry_7_projectile_allowance;
};

struct EnemyGoldCoinRFID {
  u16 enemy_gold_remaining; // 字节0-1：对方剩余金币数
  u16 enemy_gold_total; // 字节2-3：对方累计总金币数

  union {
    u32 raw;

    struct {
      // bit 0
      u16 supply_zone_occupied : 1;
      // 对方补给区占领状态

      // bit 1-2
      u16 central_highland_status : 2;
      // 对方中央高地占领状态
      // 0: 未占领
      // 1: 被对方占领
      // 2: 被己方占领

      // bit 3
      u16 trapezoidal_highland_occupied : 1;
      // 对方梯形高地占领状态
      // 1: 已占领

      // bit 4-5
      u16 fortress_buff_status : 2;
      // 对方堡垒增益点占领状态
      // 0: 未被占领
      // 1: 被对方占领
      // 2: 被己方占领
      // 3: 被双方占领

      // bit 6-7
      u16 outpost_buff_status : 2;
      // 对方前哨站增益点占领状态
      // 0: 未被占领
      // 1: 被对方占领
      // 2: 被己方占领

      // bit 8
      u16 base_buff_occupied : 1;
      // 对方基地增益点占领状态

      // bit 9
      u16 enemy_front_tunnel_detected : 1;
      // 靠近对方一侧飞坡前隧道检测到对方机器人

      // bit 10
      u16 enemy_rear_tunnel_detected : 1;
      // 靠近对方一侧飞坡后隧道检测到对方机器人

      // bit 11
      u16 friendly_front_tunnel_detected : 1;
      // 靠近己方一侧飞坡前隧道检测到对方机器人

      // bit 12
      u16 friendly_rear_tunnel_detected : 1;
      // 靠近己方一侧飞坡后隧道检测到对方机器人

      // bit 13
      u16 highland_upper_detected : 1;
      // 对方高地上部检测到对方机器人

      // bit 14
      u16 fly_slope_rear_detected : 1;
      // 对方飞坡后部检测到对方机器人

      // bit 15
      u16 road_upper_detected : 1;
      // 对方公路上部检测到对方机器人
    };
  } rfid_status;
};

struct EnemyRobotBuff {
  enum class SentryPosture : u8 {
    ATTACK = 1, // 进攻姿态
    DEFENSE = 2, // 防御姿态
    MOBILE = 3 // 移动姿态
  };

  struct RobotBuffInfo {
    u8 hp_recovery; // 回血增益（百分比）
    u16 heat_cooling; // 射击热量冷却增益（直接值）
    u8 defense; // 防御增益（百分比）
    u8 negative_defense; // 负防御增益（百分比）
    u16 attack; // 攻击增益（百分比）
  };

  // byte 0~6
  RobotBuffInfo hero;

  // byte 7~13
  RobotBuffInfo engineer;

  // byte 14~20
  RobotBuffInfo infantry3;

  // byte 21~27
  RobotBuffInfo infantry4;

  // byte 28~34
  RobotBuffInfo sentry;

  // byte 35
  SentryPosture sentry_posture;
};

struct Hero2Drone {
  f32 hero_yaw_angle;
  f32 hero_pitch_angle;
  u16 hero_projectile_allowance_42mm;
  f32 hero_initial_speed;
  u16 hero_ammo_adjust;
};

struct Drone2Hero {
  f32 cmd_yaw_angle{0};
  u16 cmd_ammo_adjust{0};
  u8 cmd_shooting{0};
};

struct RefereeSubCmdId {
  // 对应0x20B，哨兵机器人接收位置信息
  constexpr static u16 kAllyRobotPosition = 0x0200;
  // 对应雷达0x0A01~0x0A05信息波，包含敌方信息
  constexpr static u16 kEnemyRobotPosition = 0x0201;
  constexpr static u16 kEnemyRobotHP = 0x0202;
  constexpr static u16 kEnemyRobotProjectileAllowance = 0x0203;
  constexpr static u16 kEnemyGoldCoinRFID = 0x0204;
  constexpr static u16 kEnemyRobotBuff = 0x0205;
  constexpr static u16 kAllRadarInfo = 0x0206;
  // 云台手大吊射
  constexpr static u16 kHero2Drone = 0x0207;
  constexpr static u16 kDrone2Hero = 0x0208;
};
#pragma pack(pop)
template <typename T>
struct TypeToCmd;

template <>
struct TypeToCmd<AllyRobotPosition> {
  static constexpr u16 value = RefereeSubCmdId::kAllyRobotPosition;
};

template <>
struct TypeToCmd<Drone2Hero> {
  static constexpr u16 value = RefereeSubCmdId::kDrone2Hero;
};

template <>
struct TypeToCmd<Hero2Drone> {
  static constexpr u16 value = RefereeSubCmdId::kHero2Drone;
};

template <typename T>
constexpr int getCmd(const T &obj) {
  return TypeToCmd<T>::value;
}

#pragma pack(push, 1)
struct RefereeSubProtocol {
  struct AllyRobotPosition ally_robot_position; // 0x0200
  struct EnemyRobotPosition enemy_robot_position; // 0x0201
  struct EnemyRobotHP enemy_robot_HP; // 0x0202
  struct EnemyRobotProjectileAllowance enemy_robot_projectile_allowance; // 0x0203
  struct EnemyGoldCoinRFID enemy_gold_coin_RFID; // 0x0204
  struct EnemyRobotBuff enemy_robot_buff; // 0x0205
  struct Hero2Drone hero_2_drone;
  struct Drone2Hero drone_2_hero;
};
#pragma pack(pop)

struct RefereeSubProtocolMemoryMap {
  static MAPBOX_ETERNAL_CONSTEXPR const auto map = mapbox::eternal::map<u16, usize>({
      {RefereeSubCmdId::kAllyRobotPosition, offsetof(RefereeSubProtocol, ally_robot_position)},
      {RefereeSubCmdId::kEnemyRobotPosition, offsetof(RefereeSubProtocol, enemy_robot_position)},
      {RefereeSubCmdId::kEnemyRobotHP, offsetof(RefereeSubProtocol, enemy_robot_HP)},
      {RefereeSubCmdId::kEnemyRobotProjectileAllowance, offsetof(RefereeSubProtocol, enemy_robot_projectile_allowance)},
      {RefereeSubCmdId::kEnemyGoldCoinRFID, offsetof(RefereeSubProtocol, enemy_gold_coin_RFID)},
      {RefereeSubCmdId::kEnemyRobotBuff, offsetof(RefereeSubProtocol, enemy_robot_buff)},
      {RefereeSubCmdId::kAllRadarInfo, offsetof(RefereeSubProtocol, enemy_robot_HP)},
      {RefereeSubCmdId::kHero2Drone, offsetof(RefereeSubProtocol, hero_2_drone)},
      {RefereeSubCmdId::kDrone2Hero, offsetof(RefereeSubProtocol, drone_2_hero)}
  });
  static MAPBOX_ETERNAL_CONSTEXPR const auto mapSize = mapbox::eternal::map<u16, usize>({
      {RefereeSubCmdId::kAllRadarInfo,
       sizeof(RefereeSubProtocol) - sizeof(RefereeSubProtocol::ally_robot_position) - sizeof(
         RefereeSubProtocol::enemy_robot_position)},
      {RefereeSubCmdId::kAllyRobotPosition, sizeof(RefereeSubProtocol::ally_robot_position)},
      {RefereeSubCmdId::kEnemyRobotPosition, sizeof(RefereeSubProtocol::enemy_robot_position)},
      {RefereeSubCmdId::kEnemyRobotHP, sizeof(RefereeSubProtocol::enemy_robot_HP)},
      {RefereeSubCmdId::kEnemyRobotProjectileAllowance, sizeof(RefereeSubProtocol::enemy_robot_projectile_allowance)},
      {RefereeSubCmdId::kEnemyGoldCoinRFID, sizeof(RefereeSubProtocol::enemy_gold_coin_RFID)},
      {RefereeSubCmdId::kEnemyRobotBuff, sizeof(RefereeSubProtocol::enemy_robot_buff)},
      {RefereeSubCmdId::kHero2Drone, sizeof(RefereeSubProtocol::hero_2_drone)},
      {RefereeSubCmdId::kDrone2Hero, sizeof(RefereeSubProtocol::drone_2_hero)}
  });
};
} // namespace rm::device

#endif  // LIBRM_DEVICE_REFEREE_PROTOCOL_USER_HPP