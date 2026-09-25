#include <gtest/gtest.h>

#include <cstddef>
#include <format>
#include <string>

#include "drone/EmbeddedConfigs.hpp"
#include "sim/FileConfigLoader.hpp"

// Вшиті конфіги мають збігатися з тим, що дає FileConfigLoader для тих самих JSON.
// Якщо тест падає - змінено JSON але не запущено tools/gen_drone_configs.py
TEST(EmbeddedConfigs, MatchFileConfigLoaderForEveryTestCase)
{
  const std::string dataDir = HM20_DATA_DIR;

  for (std::size_t i = 0; i < kEmbeddedDroneConfigs.size(); i++) {
    const EmbeddedDroneConfig& embedded = kEmbeddedDroneConfigs[i];

    const std::string configPath = std::format("{}/testing_data/{}/{:02}_config.json", dataDir, embedded.name, i + 1);

    FileConfigLoader loader;
    ASSERT_TRUE(loader.load(configPath, dataDir + "/ammo.json")) << configPath;
    const DroneConfig expected = loader.getConfig();

    SCOPED_TRACE(embedded.name);
    EXPECT_EQ(embedded.drone.startPos.x, expected.startPos.x);
    EXPECT_EQ(embedded.drone.startPos.y, expected.startPos.y);
    EXPECT_EQ(embedded.drone.altitude, expected.altitude);
    EXPECT_EQ(embedded.drone.initialDir, expected.initialDir);
    EXPECT_EQ(embedded.drone.v0, expected.v0);
    EXPECT_EQ(embedded.drone.accelerationPath, expected.accelerationPath);
    EXPECT_EQ(embedded.drone.simTimeStep, expected.simTimeStep);
    EXPECT_EQ(embedded.drone.hitRadius, expected.hitRadius);
    EXPECT_EQ(embedded.drone.angularSpeed, expected.angularSpeed);
    EXPECT_EQ(embedded.drone.turnThreshold, expected.turnThreshold);
    EXPECT_EQ(embedded.physicsTimeStep, loader.getPhysicsTimeStep());
  }
}

// Кожен вшитий конфіг відповідає тесту з тим самим номером
TEST(EmbeddedConfigs, NamesStartWithTestNumber)
{
  for (std::size_t i = 0; i < kEmbeddedDroneConfigs.size(); i++) {
    const std::string prefix = std::format("{:02}_", i + 1);

    EXPECT_EQ(std::string(kEmbeddedDroneConfigs[i].name).rfind(prefix, 0), 0U) << kEmbeddedDroneConfigs[i].name;
  }
}
