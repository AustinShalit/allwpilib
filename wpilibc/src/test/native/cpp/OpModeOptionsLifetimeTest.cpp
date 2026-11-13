// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <algorithm>
#include <string>
#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "wpi/driverstation/RobotState.hpp"
#include "wpi/hal/DriverStationTypes.h"
#include "wpi/simulation/DriverStationSim.hpp"
#include "wpi/util/string.hpp"

namespace {
class OpModeOptionsLifetimeTest {
 public:
  OpModeOptionsLifetimeTest() {
    wpi::sim::DriverStationSim::ResetData();
    wpi::RobotState::ClearOpModes();
  }

  ~OpModeOptionsLifetimeTest() { wpi::RobotState::ClearOpModes(); }
};

const HAL_OpModeOption* FindOption(const wpi::sim::OpModeOptions& options,
                                   std::string_view name) {
  for (const auto& option : options) {
    if (wpi::util::to_string_view(&option.name) == name) {
      return &option;
    }
  }
  return nullptr;
}

// Overwrite the string in place so a stored non-owning view would observe the
// new contents without necessarily reallocating the buffer.
void OverwriteInPlace(std::string& str, char fill) {
  std::fill(str.begin(), str.end(), fill);
}
}  // namespace

TEST_CASE_METHOD(OpModeOptionsLifetimeTest,
                 "RobotState op mode options outlive source strings",
                 "[wpilibc]") {
  // Use strings longer than the typical SSO buffer so a dangling view is more
  // likely to see overwritten or freed heap memory after the source is gone.
  constexpr std::string_view kTeleopName = "TemporaryTeleopOpMode";
  constexpr std::string_view kTeleopGroup = "TemporaryTeleopGroup";
  constexpr std::string_view kTeleopDescription = "TemporaryTeleopDescription";
  constexpr std::string_view kAutoName = "TemporaryAutoOpMode";
  constexpr std::string_view kAutoGroup = "TemporaryAutoGroup";
  constexpr std::string_view kAutoDescription = "TemporaryAutoDescription";

  int64_t teleopId = 0;
  int64_t autoId = 0;
  {
    std::string name{kTeleopName};
    std::string group{kTeleopGroup};
    std::string description{kTeleopDescription};
    teleopId = wpi::RobotState::AddOpMode(wpi::RobotMode::TELEOPERATED, name,
                                          group, description);
    OverwriteInPlace(name, 'X');
    OverwriteInPlace(group, 'Y');
    OverwriteInPlace(description, 'Z');
  }
  {
    std::string name{kAutoName};
    std::string group{kAutoGroup};
    std::string description{kAutoDescription};
    autoId = wpi::RobotState::AddOpMode(wpi::RobotMode::AUTONOMOUS, name, group,
                                        description);
    OverwriteInPlace(name, 'A');
    OverwriteInPlace(group, 'B');
    OverwriteInPlace(description, 'C');
  }

  REQUIRE(teleopId != 0);
  REQUIRE(autoId != 0);
  REQUIRE(teleopId != autoId);

  {
    std::string duplicate{kTeleopName};
    CHECK(wpi::RobotState::AddOpMode(wpi::RobotMode::TELEOPERATED, duplicate) ==
          0);
    OverwriteInPlace(duplicate, 'Q');
  }

  wpi::RobotState::PublishOpModes();

  {
    auto options = wpi::sim::DriverStationSim::GetOpModeOptions();
    REQUIRE(options.size() == 2u);

    const auto* teleop = FindOption(options, kTeleopName);
    REQUIRE(teleop != nullptr);
    CHECK(teleop->id == teleopId);
    CHECK(wpi::util::to_string_view(&teleop->group) == kTeleopGroup);
    CHECK(wpi::util::to_string_view(&teleop->description) ==
          kTeleopDescription);
    CHECK(teleop->textColor == -1);
    CHECK(teleop->backgroundColor == -1);

    const auto* autonomous = FindOption(options, kAutoName);
    REQUIRE(autonomous != nullptr);
    CHECK(autonomous->id == autoId);
    CHECK(wpi::util::to_string_view(&autonomous->group) == kAutoGroup);
    CHECK(wpi::util::to_string_view(&autonomous->description) ==
          kAutoDescription);
  }

  {
    std::string toRemove{kTeleopName};
    CHECK(wpi::RobotState::RemoveOpMode(wpi::RobotMode::TELEOPERATED,
                                        toRemove) == teleopId);
    OverwriteInPlace(toRemove, 'R');
  }

  wpi::RobotState::PublishOpModes();

  {
    auto options = wpi::sim::DriverStationSim::GetOpModeOptions();
    REQUIRE(options.size() == 1u);
    CHECK(wpi::util::to_string_view(&options[0].name) == kAutoName);
    CHECK(options[0].id == autoId);
    CHECK(wpi::util::to_string_view(&options[0].group) == kAutoGroup);
    CHECK(wpi::util::to_string_view(&options[0].description) ==
          kAutoDescription);
  }
}
