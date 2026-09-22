# insta360_geometry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Реализовать пакет `insta360_geometry` — биективное отображение пикселя на сыром dual-fisheye кадре Insta360 в единичный кватернион (body frame) и обратно, с ROS-узлом визуализации и CLI.

**Architecture:** Ядро — чистая C++17 библиотека без `rclcpp` (Eigen + OpenCV для типов), модель проекции за интерфейсом `FisheyeModel` (первая реализация — экдистантная). ROS-узел и CLI — тонкие обёртки, зависящие от ядра. Тесты — gtest на ядро, без ROS-рантайма.

**Tech Stack:** C++17, ROS2 Humble, ament_cmake, Eigen3, OpenCV, gtest, `image_transport`, `cv_bridge`.

**Spec:** `docs/superpowers/specs/2026-09-22-insta360-geometry-design.md`

## Global Constraints

- C++17, ROS2 Humble, ament_cmake.
- Ядро (`lens_params`, `fisheye_model`, `equidistant_model`, `fisheye_camera`) **не зависит от `rclcpp`**. Зависимости ядра — только Eigen и OpenCV.
- Направление зависимостей: `ros/`, `cli/` → ядро. Никогда наоборот.
- Типы: `Eigen::Quaterniond` / `Eigen::Vector3d` для кватерниона/луча, `cv::Point2d` для пикселя.
- Обработка ошибок: `std::optional` при пикселе вне fisheye-круга / луче вне кадра; исключение (`std::invalid_argument`) только в конструкторе при невалидных параметрах.
- Опорная ось кватерниона: `+X` body.
- Выбор объектива: `u < frame_width/2` → front, иначе rear.
- Кадр BGR8 (не RGB): красный = `(0, 0, 255)`.
- `frame_width` по умолчанию 1920 (совпадает с `main.cpp:130` драйвера).
- Тесты запускаются на ноутбуке, не на Orange Pi.
- C++ targets объявляются явно в `CMakeLists.txt` (без globbing).
- Коммиты — только явные пути файлов, никогда `git add -A`.

---

### Task 1: Скелет пакета и сборка

**Files:**
- Create: `insta360_geometry/package.xml`
- Create: `insta360_geometry/CMakeLists.txt`
- Create: `insta360_geometry/include/insta360_geometry/lens_params.hpp`
- Create: `insta360_geometry/test/test_lens_params.cpp`

**Interfaces:**
- Consumes: ничего.
- Produces: пакет `insta360_geometry`, собираемый через colcon; `struct LensParams { double fx, fy, cx, cy; }` в `insta360_geometry/lens_params.hpp`.

- [ ] **Step 1: Создать `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>insta360_geometry</name>
  <version>0.1.0</version>
  <description>Pixel to unit quaternion bijection for Insta360 dual-fisheye frames</description>
  <maintainer email="samfrompizza@users.noreply.github.com">samfrompizza</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>eigen</depend>
  <depend>libopencv-dev</depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>image_transport</depend>
  <depend>cv_bridge</depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Создать `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(insta360_geometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# --- Core library (no rclcpp) ---
add_library(insta360_geometry_core
  src/equidistant_model.cpp
  src/fisheye_camera.cpp
)
target_include_directories(insta360_geometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(insta360_geometry_core PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

install(TARGETS insta360_geometry_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lens_params test/test_lens_params.cpp)
  target_link_libraries(test_lens_params insta360_geometry_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(insta360_geometry_core)
ament_export_dependencies(Eigen3 OpenCV)
ament_package()
```

- [ ] **Step 3: Создать `include/insta360_geometry/lens_params.hpp`**

```cpp
#ifndef INSTA360_GEOMETRY__LENS_PARAMS_HPP_
#define INSTA360_GEOMETRY__LENS_PARAMS_HPP_

namespace insta360_geometry
{

/// Intrinsic parameters of a single fisheye lens, in pixels.
struct LensParams
{
  double fx;
  double fy;
  double cx;
  double cy;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__LENS_PARAMS_HPP_
```

- [ ] **Step 4: Создать заглушки `src/equidistant_model.cpp` и `src/fisheye_camera.cpp`**

Оба файла пока пустые (только include собственного заголовка), чтобы библиотека собиралась. Заголовки `equidistant_model.hpp` и `fisheye_camera.hpp` создаются в Task 3 и Task 5; на этом шаге создаём минимальные заголовки-заглушки, чтобы `add_library` не падал.

`include/insta360_geometry/equidistant_model.hpp`:
```cpp
#ifndef INSTA360_GEOMETRY__EQUIDISTANT_MODEL_HPP_
#define INSTA360_GEOMETRY__EQUIDISTANT_MODEL_HPP_
#endif  // INSTA360_GEOMETRY__EQUIDISTANT_MODEL_HPP_
```

`include/insta360_geometry/fisheye_camera.hpp`:
```cpp
#ifndef INSTA360_GEOMETRY__FISHEYE_CAMERA_HPP_
#define INSTA360_GEOMETRY__FISHEYE_CAMERA_HPP_
#endif  // INSTA360_GEOMETRY__FISHEYE_CAMERA_HPP_
```

`src/equidistant_model.cpp`:
```cpp
#include "insta360_geometry/equidistant_model.hpp"
```

`src/fisheye_camera.cpp`:
```cpp
#include "insta360_geometry/fisheye_camera.hpp"
```

- [ ] **Step 5: Создать `test/test_lens_params.cpp`**

```cpp
#include <gtest/gtest.h>

#include "insta360_geometry/lens_params.hpp"

TEST(LensParams, HoldsIntrinsics)
{
  insta360_geometry::LensParams p{480.0, 480.0, 480.0, 480.0};
  EXPECT_DOUBLE_EQ(p.fx, 480.0);
  EXPECT_DOUBLE_EQ(p.fy, 480.0);
  EXPECT_DOUBLE_EQ(p.cx, 480.0);
  EXPECT_DOUBLE_EQ(p.cy, 480.0);
}
```

- [ ] **Step 6: Собрать и запустить тест**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
colcon test --packages-select insta360_geometry
colcon test-result --verbose
```
Expected: сборка успешна, `test_lens_params` PASS.

- [ ] **Step 7: Commit**

```bash
git add insta360_geometry/package.xml insta360_geometry/CMakeLists.txt \
  insta360_geometry/include/insta360_geometry/lens_params.hpp \
  insta360_geometry/include/insta360_geometry/equidistant_model.hpp \
  insta360_geometry/include/insta360_geometry/fisheye_camera.hpp \
  insta360_geometry/src/equidistant_model.cpp \
  insta360_geometry/src/fisheye_camera.cpp \
  insta360_geometry/test/test_lens_params.cpp
git commit -m "feat(geometry): package skeleton with core library and gtest"
```

---

### Task 2: Интерфейс `FisheyeModel`

**Files:**
- Create: `insta360_geometry/include/insta360_geometry/fisheye_model.hpp`
- Modify: `insta360_geometry/CMakeLists.txt` (добавить `test_fisheye_model`)

**Interfaces:**
- Consumes: `LensParams` (Task 1).
- Produces: абстрактный класс `FisheyeModel` с чисто виртуальными `pixelToRay(const cv::Point2d&, const LensParams&) const -> std::optional<Eigen::Vector3d>` и `rayToPixel(const Eigen::Vector3d&, const LensParams&) const -> std::optional<cv::Point2d>`; виртуальный деструктор.

- [ ] **Step 1: Создать `include/insta360_geometry/fisheye_model.hpp`**

```cpp
#ifndef INSTA360_GEOMETRY__FISHEYE_MODEL_HPP_
#define INSTA360_GEOMETRY__FISHEYE_MODEL_HPP_

#include <optional>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

#include "insta360_geometry/lens_params.hpp"

namespace insta360_geometry
{

/// Projection model for a fisheye lens.
///
/// Implementations map between a pixel and a unit ray expressed in the lens
/// optical frame (Z along the optical axis, X right, Y down).
class FisheyeModel
{
public:
  virtual ~FisheyeModel() = default;

  /// Pixel -> unit ray in the lens optical frame.
  /// Returns nullopt if the pixel lies outside the fisheye image circle.
  virtual std::optional<Eigen::Vector3d> pixelToRay(
    const cv::Point2d & pixel, const LensParams & params) const = 0;

  /// Unit ray in the lens optical frame -> pixel.
  /// Returns nullopt if the ray falls outside the lens field of view.
  virtual std::optional<cv::Point2d> rayToPixel(
    const Eigen::Vector3d & ray, const LensParams & params) const = 0;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__FISHEYE_MODEL_HPP_
```

- [ ] **Step 2: Добавить тест-таргет в `CMakeLists.txt`**

Внутри блока `if(BUILD_TESTING)` добавить:
```cmake
  ament_add_gtest(test_fisheye_model test/test_fisheye_model.cpp)
  target_link_libraries(test_fisheye_model insta360_geometry_core)
```

- [ ] **Step 3: Создать `test/test_fisheye_model.cpp`**

Тест проверяет, что интерфейс полиморфен и удаляется через базовый указатель (проверка виртуального деструктора).

```cpp
#include <gtest/gtest.h>

#include <memory>

#include "insta360_geometry/fisheye_model.hpp"

namespace
{

class StubModel : public insta360_geometry::FisheyeModel
{
public:
  std::optional<Eigen::Vector3d> pixelToRay(
    const cv::Point2d &, const insta360_geometry::LensParams &) const override
  {
    return Eigen::Vector3d::UnitZ();
  }

  std::optional<cv::Point2d> rayToPixel(
    const Eigen::Vector3d &, const insta360_geometry::LensParams &) const override
  {
    return cv::Point2d(0.0, 0.0);
  }
};

}  // namespace

TEST(FisheyeModel, PolymorphicDeletion)
{
  std::unique_ptr<insta360_geometry::FisheyeModel> model = std::make_unique<StubModel>();
  insta360_geometry::LensParams p{1.0, 1.0, 0.0, 0.0};
  auto ray = model->pixelToRay(cv::Point2d(0.0, 0.0), p);
  ASSERT_TRUE(ray.has_value());
  EXPECT_DOUBLE_EQ(ray->z(), 1.0);
}
```

- [ ] **Step 4: Собрать и запустить тест**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
colcon test --packages-select insta360_geometry
colcon test-result --verbose
```
Expected: `test_fisheye_model` PASS.

- [ ] **Step 5: Commit**

```bash
git add insta360_geometry/include/insta360_geometry/fisheye_model.hpp \
  insta360_geometry/test/test_fisheye_model.cpp \
  insta360_geometry/CMakeLists.txt
git commit -m "feat(geometry): add FisheyeModel interface"
```

---

### Task 3: `EquidistantModel` — pixelToRay

**Files:**
- Modify: `insta360_geometry/include/insta360_geometry/equidistant_model.hpp`
- Modify: `insta360_geometry/src/equidistant_model.cpp`
- Create: `insta360_geometry/test/test_equidistant_model.cpp`
- Modify: `insta360_geometry/CMakeLists.txt` (добавить `test_equidistant_model`)

**Interfaces:**
- Consumes: `FisheyeModel` (Task 2), `LensParams` (Task 1).
- Produces: класс `EquidistantModel : public FisheyeModel` с реализованным `pixelToRay`; `rayToPixel` пока возвращает `std::nullopt` (реализуется в Task 4).

**Математика (экдистантная модель):**
- Прямое: `dx = u - cx`, `dy = v - cy`, `r = sqrt(dx² + dy²)`, `θ = r / fx` (при `fx == fy`; при разных — использовать `fx` для `r` по X и `fy` по Y, см. реализацию ниже). Луч в оптической системе: `x = sin θ · dx/r`, `y = sin θ · dy/r`, `z = cos θ`. При `r == 0` луч = `(0, 0, 1)`.
- Граница круга: `r > f · π/2` → `nullopt` (FOV 180°).

- [ ] **Step 1: Написать падающий тест `test/test_equidistant_model.cpp`**

```cpp
#include <gtest/gtest.h>

#include <cmath>

#include "insta360_geometry/equidistant_model.hpp"

using insta360_geometry::EquidistantModel;
using insta360_geometry::LensParams;

namespace
{
const LensParams kParams{480.0, 480.0, 480.0, 480.0};
}

TEST(EquidistantModel, CenterPixelMapsToOpticalAxis)
{
  EquidistantModel model;
  auto ray = model.pixelToRay(cv::Point2d(480.0, 480.0), kParams);
  ASSERT_TRUE(ray.has_value());
  EXPECT_NEAR(ray->x(), 0.0, 1e-12);
  EXPECT_NEAR(ray->y(), 0.0, 1e-12);
  EXPECT_NEAR(ray->z(), 1.0, 1e-12);
}

TEST(EquidistantModel, KnownRadiusGivesKnownAngle)
{
  EquidistantModel model;
  // r = 240 px, f = 480 px -> theta = 0.5 rad, along +X
  auto ray = model.pixelToRay(cv::Point2d(720.0, 480.0), kParams);
  ASSERT_TRUE(ray.has_value());
  EXPECT_NEAR(ray->x(), std::sin(0.5), 1e-12);
  EXPECT_NEAR(ray->y(), 0.0, 1e-12);
  EXPECT_NEAR(ray->z(), std::cos(0.5), 1e-12);
}

TEST(EquidistantModel, RayIsUnitLength)
{
  EquidistantModel model;
  auto ray = model.pixelToRay(cv::Point2d(600.0, 400.0), kParams);
  ASSERT_TRUE(ray.has_value());
  EXPECT_NEAR(ray->norm(), 1.0, 1e-12);
}

TEST(EquidistantModel, PixelOutsideCircleReturnsNullopt)
{
  EquidistantModel model;
  // r = 480*sqrt(2) ~ 679 px > f*pi/2 ~ 754? No: 679 < 754, so use a corner
  // far enough: r must exceed 754 px. Use (480 + 800, 480).
  auto ray = model.pixelToRay(cv::Point2d(1280.0, 480.0), kParams);
  EXPECT_FALSE(ray.has_value());
}
```

- [ ] **Step 2: Запустить тест — убедиться, что падает**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
```
Expected: FAIL компиляции — `EquidistantModel` не определён.

- [ ] **Step 3: Реализовать `equidistant_model.hpp`**

```cpp
#ifndef INSTA360_GEOMETRY__EQUIDISTANT_MODEL_HPP_
#define INSTA360_GEOMETRY__EQUIDISTANT_MODEL_HPP_

#include <optional>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

#include "insta360_geometry/fisheye_model.hpp"
#include "insta360_geometry/lens_params.hpp"

namespace insta360_geometry
{

/// Equidistant fisheye projection: r = f * theta.
class EquidistantModel : public FisheyeModel
{
public:
  std::optional<Eigen::Vector3d> pixelToRay(
    const cv::Point2d & pixel, const LensParams & params) const override;

  std::optional<cv::Point2d> rayToPixel(
    const Eigen::Vector3d & ray, const LensParams & params) const override;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__EQUIDISTANT_MODEL_HPP_
```

- [ ] **Step 4: Реализовать `pixelToRay` в `src/equidistant_model.cpp`**

```cpp
#include "insta360_geometry/equidistant_model.hpp"

#include <cmath>

namespace insta360_geometry
{

std::optional<Eigen::Vector3d> EquidistantModel::pixelToRay(
  const cv::Point2d & pixel, const LensParams & params) const
{
  const double dx = pixel.x - params.cx;
  const double dy = pixel.y - params.cy;
  const double r = std::hypot(dx, dy);

  // FOV limit: theta_max = pi/2 -> r_max = f * pi/2.
  const double r_max = params.fx * (M_PI / 2.0);
  if (r > r_max) {
    return std::nullopt;
  }

  if (r == 0.0) {
    return Eigen::Vector3d(0.0, 0.0, 1.0);
  }

  const double theta = r / params.fx;
  const double sin_theta = std::sin(theta);
  return Eigen::Vector3d(
    sin_theta * dx / r,
    sin_theta * dy / r,
    std::cos(theta));
}

std::optional<cv::Point2d> EquidistantModel::rayToPixel(
  const Eigen::Vector3d &, const LensParams &) const
{
  return std::nullopt;  // implemented in Task 4
}

}  // namespace insta360_geometry
```

- [ ] **Step 5: Добавить тест-таргет в `CMakeLists.txt`**

Внутри `if(BUILD_TESTING)`:
```cmake
  ament_add_gtest(test_equidistant_model test/test_equidistant_model.cpp)
  target_link_libraries(test_equidistant_model insta360_geometry_core)
```

- [ ] **Step 6: Собрать и запустить тесты**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
colcon test --packages-select insta360_geometry
colcon test-result --verbose
```
Expected: `test_equidistant_model` PASS (4 теста).

- [ ] **Step 7: Commit**

```bash
git add insta360_geometry/include/insta360_geometry/equidistant_model.hpp \
  insta360_geometry/src/equidistant_model.cpp \
  insta360_geometry/test/test_equidistant_model.cpp \
  insta360_geometry/CMakeLists.txt
git commit -m "feat(geometry): equidistant pixelToRay"
```

---

### Task 4: `EquidistantModel` — rayToPixel и round-trip

**Files:**
- Modify: `insta360_geometry/src/equidistant_model.cpp`
- Modify: `insta360_geometry/test/test_equidistant_model.cpp`

**Interfaces:**
- Consumes: `EquidistantModel::pixelToRay` (Task 3).
- Produces: реализованный `EquidistantModel::rayToPixel`.

**Математика (обратное):**
- `θ = acos(clamp(z, -1, 1))`; если `θ > π/2` → `nullopt`.
- `r = fx · θ`; `φ = atan2(y, x)`; `u = cx + r·cos φ`, `v = cy + r·sin φ`.

- [ ] **Step 1: Добавить падающие тесты round-trip**

Добавить в `test/test_equidistant_model.cpp`:

```cpp
TEST(EquidistantModel, RayToPixelCenter)
{
  EquidistantModel model;
  auto px = model.rayToPixel(Eigen::Vector3d(0.0, 0.0, 1.0), kParams);
  ASSERT_TRUE(px.has_value());
  EXPECT_NEAR(px->x, 480.0, 1e-9);
  EXPECT_NEAR(px->y, 480.0, 1e-9);
}

TEST(EquidistantModel, RayToPixelKnownAngle)
{
  EquidistantModel model;
  // theta = 0.5 rad along +X -> r = 240 px -> u = 720
  Eigen::Vector3d ray(std::sin(0.5), 0.0, std::cos(0.5));
  auto px = model.rayToPixel(ray, kParams);
  ASSERT_TRUE(px.has_value());
  EXPECT_NEAR(px->x, 720.0, 1e-9);
  EXPECT_NEAR(px->y, 480.0, 1e-9);
}

TEST(EquidistantModel, RayOutsideFovReturnsNullopt)
{
  EquidistantModel model;
  // theta = 2.0 rad > pi/2
  Eigen::Vector3d ray(std::sin(2.0), 0.0, std::cos(2.0));
  EXPECT_FALSE(model.rayToPixel(ray, kParams).has_value());
}

TEST(EquidistantModel, RoundTripGrid)
{
  EquidistantModel model;
  for (double u = 100.0; u <= 860.0; u += 40.0) {
    for (double v = 100.0; v <= 860.0; v += 40.0) {
      auto ray = model.pixelToRay(cv::Point2d(u, v), kParams);
      if (!ray.has_value()) {
        continue;
      }
      auto px = model.rayToPixel(*ray, kParams);
      ASSERT_TRUE(px.has_value()) << "u=" << u << " v=" << v;
      EXPECT_NEAR(px->x, u, 1e-6) << "u=" << u << " v=" << v;
      EXPECT_NEAR(px->y, v, 1e-6) << "u=" << u << " v=" << v;
    }
  }
}
```

- [ ] **Step 2: Запустить — убедиться, что падает**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
colcon test --packages-select insta360_geometry
```
Expected: FAIL — `rayToPixel` возвращает `nullopt`.

- [ ] **Step 3: Реализовать `rayToPixel`**

Заменить заглушку в `src/equidistant_model.cpp`:

```cpp
std::optional<cv::Point2d> EquidistantModel::rayToPixel(
  const Eigen::Vector3d & ray, const LensParams & params) const
{
  const double norm = ray.norm();
  if (norm == 0.0) {
    return std::nullopt;
  }
  const Eigen::Vector3d unit = ray / norm;

  const double z = std::clamp(unit.z(), -1.0, 1.0);
  const double theta = std::acos(z);
  if (theta > M_PI / 2.0) {
    return std::nullopt;
  }

  const double r = params.fx * theta;
  const double phi = std::atan2(unit.y(), unit.x());
  return cv::Point2d(
    params.cx + r * std::cos(phi),
    params.cy + r * std::sin(phi));
}
```

Добавить `#include <algorithm>` в начало файла (для `std::clamp`).

- [ ] **Step 4: Собрать и запустить тесты**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
colcon test --packages-select insta360_geometry
colcon test-result --verbose
```
Expected: все тесты `test_equidistant_model` PASS (8 тестов).

- [ ] **Step 5: Commit**

```bash
git add insta360_geometry/src/equidistant_model.cpp \
  insta360_geometry/test/test_equidistant_model.cpp
git commit -m "feat(geometry): equidistant rayToPixel and round-trip tests"
```

---

### Task 5: `FisheyeCamera` — конструктор и выбор объектива

**Files:**
- Modify: `insta360_geometry/include/insta360_geometry/fisheye_camera.hpp`
- Modify: `insta360_geometry/src/fisheye_camera.cpp`
- Create: `insta360_geometry/test/test_fisheye_camera.cpp`
- Modify: `insta360_geometry/CMakeLists.txt` (добавить `test_fisheye_camera`)

**Interfaces:**
- Consumes: `LensParams` (Task 1), `FisheyeModel` (Task 2).
- Produces: класс `FisheyeCamera` с конструктором `FisheyeCamera(LensParams front, LensParams rear, int frame_width, std::unique_ptr<const FisheyeModel> model)`; `pixelToQuaternion` и `quaternionToPixel` пока заглушки (Task 6).

**Геометрия body frame:**
- Front lens смотрит в `+X`, rear — в `−X`.
- Оптическая система front: `Z_opt = +X_body`, `X_opt = −Y_body`, `Y_opt = −Z_body` (правая тройка).
- Оптическая система rear: `Z_opt = −X_body`, `X_opt = +Y_body`, `Y_opt = −Z_body`.

- [ ] **Step 1: Написать падающий тест конструктора**

`test/test_fisheye_camera.cpp`:

```cpp
#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

#include "insta360_geometry/equidistant_model.hpp"
#include "insta360_geometry/fisheye_camera.hpp"

using insta360_geometry::EquidistantModel;
using insta360_geometry::FisheyeCamera;
using insta360_geometry::LensParams;

namespace
{
const LensParams kFront{480.0, 480.0, 480.0, 480.0};
const LensParams kRear{480.0, 480.0, 480.0, 480.0};

std::unique_ptr<const insta360_geometry::FisheyeModel> makeModel()
{
  return std::make_unique<EquidistantModel>();
}
}  // namespace

TEST(FisheyeCamera, RejectsNonPositiveFocalLength)
{
  LensParams bad{0.0, 480.0, 480.0, 480.0};
  EXPECT_THROW(
    FisheyeCamera(bad, kRear, 1920, makeModel()),
    std::invalid_argument);
}

TEST(FisheyeCamera, RejectsNonPositiveFrameWidth)
{
  EXPECT_THROW(
    FisheyeCamera(kFront, kRear, 0, makeModel()),
    std::invalid_argument);
}

TEST(FisheyeCamera, ConstructsWithValidParams)
{
  EXPECT_NO_THROW(FisheyeCamera(kFront, kRear, 1920, makeModel()));
}
```

- [ ] **Step 2: Запустить — убедиться, что падает**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
```
Expected: FAIL компиляции — `FisheyeCamera` не определён.

- [ ] **Step 3: Реализовать `fisheye_camera.hpp`**

```cpp
#ifndef INSTA360_GEOMETRY__FISHEYE_CAMERA_HPP_
#define INSTA360_GEOMETRY__FISHEYE_CAMERA_HPP_

#include <memory>
#include <optional>

#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>

#include "insta360_geometry/fisheye_model.hpp"
#include "insta360_geometry/lens_params.hpp"

namespace insta360_geometry
{

/// Dual-fisheye camera: maps pixels of a side-by-side dual-fisheye frame to
/// unit quaternions in the camera body frame, and back.
///
/// Body frame: X along the lens axis (front lens -> +X, rear lens -> -X),
/// Y and Z perpendicular, right-handed.
class FisheyeCamera
{
public:
  FisheyeCamera(
    LensParams front, LensParams rear, int frame_width,
    std::unique_ptr<const FisheyeModel> model);

  /// Pixel -> rotation quaternion (A1): rotates +X body onto the ray direction.
  std::optional<Eigen::Quaterniond> pixelToQuaternion(const cv::Point2d & pixel) const;

  /// Rotation quaternion -> pixel.
  std::optional<cv::Point2d> quaternionToPixel(const Eigen::Quaterniond & q) const;

private:
  LensParams front_;
  LensParams rear_;
  int frame_width_;
  std::unique_ptr<const FisheyeModel> model_;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__FISHEYE_CAMERA_HPP_
```

- [ ] **Step 4: Реализовать конструктор в `src/fisheye_camera.cpp`**

```cpp
#include "insta360_geometry/fisheye_camera.hpp"

#include <stdexcept>

namespace insta360_geometry
{

FisheyeCamera::FisheyeCamera(
  LensParams front, LensParams rear, int frame_width,
  std::unique_ptr<const FisheyeModel> model)
: front_(front), rear_(rear), frame_width_(frame_width), model_(std::move(model))
{
  if (front_.fx <= 0.0 || front_.fy <= 0.0) {
    throw std::invalid_argument("FisheyeCamera: front focal length must be positive");
  }
  if (rear_.fx <= 0.0 || rear_.fy <= 0.0) {
    throw std::invalid_argument("FisheyeCamera: rear focal length must be positive");
  }
  if (frame_width_ <= 0) {
    throw std::invalid_argument("FisheyeCamera: frame_width must be positive");
  }
  if (!model_) {
    throw std::invalid_argument("FisheyeCamera: model must not be null");
  }
}

std::optional<Eigen::Quaterniond> FisheyeCamera::pixelToQuaternion(
  const cv::Point2d &) const
{
  return std::nullopt;  // implemented in Task 6
}

std::optional<cv::Point2d> FisheyeCamera::quaternionToPixel(
  const Eigen::Quaterniond &) const
{
  return std::nullopt;  // implemented in Task 6
}

}  // namespace insta360_geometry
```

- [ ] **Step 5: Добавить тест-таргет в `CMakeLists.txt`**

Внутри `if(BUILD_TESTING)`:
```cmake
  ament_add_gtest(test_fisheye_camera test/test_fisheye_camera.cpp)
  target_link_libraries(test_fisheye_camera insta360_geometry_core)
```

- [ ] **Step 6: Собрать и запустить тесты**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
colcon test --packages-select insta360_geometry
colcon test-result --verbose
```
Expected: `test_fisheye_camera` PASS (3 теста).

- [ ] **Step 7: Commit**

```bash
git add insta360_geometry/include/insta360_geometry/fisheye_camera.hpp \
  insta360_geometry/src/fisheye_camera.cpp \
  insta360_geometry/test/test_fisheye_camera.cpp \
  insta360_geometry/CMakeLists.txt
git commit -m "feat(geometry): FisheyeCamera constructor and validation"
```

---

### Task 6: `FisheyeCamera` — pixelToQuaternion / quaternionToPixel

**Files:**
- Modify: `insta360_geometry/src/fisheye_camera.cpp`
- Modify: `insta360_geometry/test/test_fisheye_camera.cpp`

**Interfaces:**
- Consumes: `FisheyeCamera` (Task 5), `EquidistantModel` (Tasks 3–4).
- Produces: реализованные `pixelToQuaternion` / `quaternionToPixel`.

**Математика:**
- Выбор объектива: `pixel.x < frame_width_/2` → front, иначе rear.
- `pixelToRay` даёт луч в оптической системе объектива. Перевести в body:
  - front: `ray_body = (−ray_opt.y, −ray_opt.x, ray_opt.z)`? Нет — использовать матрицу. Оптическая система front: `X_opt = −Y_body`, `Y_opt = −Z_body`, `Z_opt = +X_body`. Значит `ray_body = ray_opt.x·X_opt + ray_opt.y·Y_opt + ray_opt.z·Z_opt = (−ray_opt.x, −ray_opt.y, ray_opt.z)` в координатах body? Проверить: `X_opt` в body = `(0,−1,0)`, `Y_opt` в body = `(0,0,−1)`, `Z_opt` в body = `(1,0,0)`. Тогда `ray_body = ray_opt.x·(0,−1,0) + ray_opt.y·(0,0,−1) + ray_opt.z·(1,0,0) = (ray_opt.z, −ray_opt.x, −ray_opt.y)`.
  - rear: `X_opt` в body = `(0,1,0)`, `Y_opt` в body = `(0,0,−1)`, `Z_opt` в body = `(−1,0,0)`. Тогда `ray_body = (−ray_opt.z, ray_opt.x, −ray_opt.y)`.
- Кватернион A1: `q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), ray_body)`.
- Обратно: `ray_body = q * Eigen::Vector3d::UnitX()`; затем в оптическую систему объектива (обратные матрицы выше) и `rayToPixel`.

- [ ] **Step 1: Добавить падающие тесты**

Добавить в `test/test_fisheye_camera.cpp`:

```cpp
TEST(FisheyeCamera, FrontCenterPixelGivesIdentityRotation)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  // Front lens center is at (480, 480); ray along +X body -> identity quaternion.
  auto q = cam.pixelToQuaternion(cv::Point2d(480.0, 480.0));
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(std::abs(q->w()), 1.0, 1e-9);
  EXPECT_NEAR(q->x(), 0.0, 1e-9);
  EXPECT_NEAR(q->y(), 0.0, 1e-9);
  EXPECT_NEAR(q->z(), 0.0, 1e-9);
}

TEST(FisheyeCamera, RearCenterPixelGives180Rotation)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  // Rear lens center is at (1440, 480); ray along -X body.
  auto q = cam.pixelToQuaternion(cv::Point2d(1440.0, 480.0));
  ASSERT_TRUE(q.has_value());
  Eigen::Vector3d axis = q->toRotationMatrix() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(axis.x(), -1.0, 1e-9);
  EXPECT_NEAR(axis.y(), 0.0, 1e-9);
  EXPECT_NEAR(axis.z(), 0.0, 1e-9);
}

TEST(FisheyeCamera, QuaternionIsNormalized)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  auto q = cam.pixelToQuaternion(cv::Point2d(600.0, 400.0));
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(q->norm(), 1.0, 1e-12);
}

TEST(FisheyeCamera, ReferenceAxisMapsToRay)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  auto q = cam.pixelToQuaternion(cv::Point2d(600.0, 400.0));
  ASSERT_TRUE(q.has_value());
  Eigen::Vector3d ray = q->toRotationMatrix() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(ray.norm(), 1.0, 1e-12);
}

TEST(FisheyeCamera, RoundTripFrontAndRear)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  const double pixels[][2] = {
    {480.0, 480.0}, {600.0, 400.0}, {300.0, 700.0},
    {1440.0, 480.0}, {1300.0, 600.0}, {1600.0, 300.0},
  };
  for (const auto & p : pixels) {
    auto q = cam.pixelToQuaternion(cv::Point2d(p[0], p[1]));
    ASSERT_TRUE(q.has_value()) << "u=" << p[0] << " v=" << p[1];
    auto px = cam.quaternionToPixel(*q);
    ASSERT_TRUE(px.has_value()) << "u=" << p[0] << " v=" << p[1];
    EXPECT_NEAR(px->x, p[0], 1e-6) << "u=" << p[0] << " v=" << p[1];
    EXPECT_NEAR(px->y, p[1], 1e-6) << "u=" << p[0] << " v=" << p[1];
  }
}

TEST(FisheyeCamera, PixelOutsideCircleReturnsNullopt)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  // Front lens: r = 800 px > f*pi/2 ~ 754 px
  EXPECT_FALSE(cam.pixelToQuaternion(cv::Point2d(1280.0, 480.0)).has_value());
}
```

- [ ] **Step 2: Запустить — убедиться, что падает**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
colcon test --packages-select insta360_geometry
```
Expected: FAIL — `pixelToQuaternion` возвращает `nullopt`.

- [ ] **Step 3: Реализовать `pixelToQuaternion` и `quaternionToPixel`**

Заменить заглушки в `src/fisheye_camera.cpp`:

```cpp
namespace
{

// Optical frame -> body frame basis, expressed in body coordinates.
// Front lens: Z_opt = +X_body, X_opt = -Y_body, Y_opt = -Z_body.
Eigen::Vector3d opticalToBodyFront(const Eigen::Vector3d & r)
{
  return Eigen::Vector3d(r.z(), -r.x(), -r.y());
}

// Rear lens: Z_opt = -X_body, X_opt = +Y_body, Y_opt = -Z_body.
Eigen::Vector3d opticalToBodyRear(const Eigen::Vector3d & r)
{
  return Eigen::Vector3d(-r.z(), r.x(), -r.y());
}

// Inverse of the above: body -> optical frame.
Eigen::Vector3d bodyToOpticalFront(const Eigen::Vector3d & r)
{
  return Eigen::Vector3d(-r.y(), -r.z(), r.x());
}

Eigen::Vector3d bodyToOpticalRear(const Eigen::Vector3d & r)
{
  return Eigen::Vector3d(r.y(), -r.z(), -r.x());
}

}  // namespace

std::optional<Eigen::Quaterniond> FisheyeCamera::pixelToQuaternion(
  const cv::Point2d & pixel) const
{
  const bool is_front = pixel.x < static_cast<double>(frame_width_) / 2.0;
  const LensParams & params = is_front ? front_ : rear_;

  auto ray_opt = model_->pixelToRay(pixel, params);
  if (!ray_opt.has_value()) {
    return std::nullopt;
  }

  const Eigen::Vector3d ray_body =
    is_front ? opticalToBodyFront(*ray_opt) : opticalToBodyRear(*ray_opt);

  return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), ray_body);
}

std::optional<cv::Point2d> FisheyeCamera::quaternionToPixel(
  const Eigen::Quaterniond & q) const
{
  const Eigen::Vector3d ray_body = q * Eigen::Vector3d::UnitX();

  // Front lens covers the +X hemisphere, rear the -X hemisphere.
  const bool is_front = ray_body.x() >= 0.0;
  const LensParams & params = is_front ? front_ : rear_;

  const Eigen::Vector3d ray_opt =
    is_front ? bodyToOpticalFront(ray_body) : bodyToOpticalRear(ray_body);

  return model_->rayToPixel(ray_opt, params);
}
```

- [ ] **Step 4: Собрать и запустить тесты**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
colcon test --packages-select insta360_geometry
colcon test-result --verbose
```
Expected: все тесты PASS.

- [ ] **Step 5: Commit**

```bash
git add insta360_geometry/src/fisheye_camera.cpp \
  insta360_geometry/test/test_fisheye_camera.cpp
git commit -m "feat(geometry): pixelToQuaternion and quaternionToPixel"
```

---

### Task 7: ROS-узел `quaternion_visualizer`

**Files:**
- Create: `insta360_geometry/src/ros/quaternion_visualizer.cpp`
- Modify: `insta360_geometry/CMakeLists.txt` (добавить исполняемый файл)

**Interfaces:**
- Consumes: `FisheyeCamera` (Tasks 5–6), `EquidistantModel` (Tasks 3–4).
- Produces: исполняемый файл `quaternion_visualizer`; подписка `/dual_fisheye/image`; топик команд `~/marker_command` (`std_msgs/String`); публикация `~/visualization` (`sensor_msgs/Image`).

**Формат команд** (`std_msgs/String.data`):
- `draw w x y z` — добавить маркер.
- `delete w x y z` — удалить маркер (угловой допуск 1e-6).
- `clear` — удалить все.

- [ ] **Step 1: Реализовать узел**

`src/ros/quaternion_visualizer.cpp`:

```cpp
#include <algorithm>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "insta360_geometry/equidistant_model.hpp"
#include "insta360_geometry/fisheye_camera.hpp"

namespace
{

constexpr double kQuaternionTolerance = 1e-6;

bool quaternionsClose(
  const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  const double dot = std::abs(a.dot(b));
  return (1.0 - dot) < kQuaternionTolerance;
}

}  // namespace

class QuaternionVisualizer : public rclcpp::Node
{
public:
  QuaternionVisualizer()
  : Node("quaternion_visualizer")
  {
    const int frame_width = declare_parameter("frame_width", 1920);
    const int roi_size = declare_parameter("roi_size", 32);

    insta360_geometry::LensParams front{
      declare_parameter("front_fx", 480.0),
      declare_parameter("front_fy", 480.0),
      declare_parameter("front_cx", 480.0),
      declare_parameter("front_cy", 480.0)};
    insta360_geometry::LensParams rear{
      declare_parameter("rear_fx", 480.0),
      declare_parameter("rear_fy", 480.0),
      declare_parameter("rear_cx", 1440.0),
      declare_parameter("rear_cy", 480.0)};

    camera_ = std::make_unique<insta360_geometry::FisheyeCamera>(
      front, rear, frame_width,
      std::make_unique<insta360_geometry::EquidistantModel>());

    roi_size_ = roi_size;

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/dual_fisheye/image", rclcpp::SensorDataQoS(),
      std::bind(&QuaternionVisualizer::onImage, this, std::placeholders::_1));

    command_sub_ = create_subscription<std_msgs::msg::String>(
      "~/marker_command", 10,
      std::bind(&QuaternionVisualizer::onCommand, this, std::placeholders::_1));

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/visualization", 10);
  }

private:
  void onCommand(const std_msgs::msg::String::SharedPtr msg)
  {
    std::istringstream stream(msg->data);
    std::string action;
    stream >> action;

    if (action == "clear") {
      markers_.clear();
      return;
    }

    double w = 0.0, x = 0.0, y = 0.0, z = 0.0;
    if (!(stream >> w >> x >> y >> z)) {
      RCLCPP_WARN(get_logger(), "Malformed command: '%s'", msg->data.c_str());
      return;
    }
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize();

    if (action == "draw") {
      markers_.push_back(q);
    } else if (action == "delete") {
      markers_.erase(
        std::remove_if(
          markers_.begin(), markers_.end(),
          [&q](const Eigen::Quaterniond & m) { return quaternionsClose(m, q); }),
        markers_.end());
    } else {
      RCLCPP_WARN(get_logger(), "Unknown action: '%s'", action.c_str());
    }
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (markers_.empty()) {
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    for (const auto & q : markers_) {
      auto pixel = camera_->quaternionToPixel(q);
      if (!pixel.has_value()) {
        continue;
      }
      publishRoi(cv_ptr->image, *pixel, msg->header);
    }
  }

  void publishRoi(
    const cv::Mat & frame, const cv::Point2d & pixel,
    const std_msgs::msg::Header & header)
  {
    const int half = roi_size_ / 2;
    const int u = static_cast<int>(std::lround(pixel.x));
    const int v = static_cast<int>(std::lround(pixel.y));

    const int x0 = std::clamp(u - half, 0, std::max(0, frame.cols - roi_size_));
    const int y0 = std::clamp(v - half, 0, std::max(0, frame.rows - roi_size_));
    const int w = std::min(roi_size_, frame.cols - x0);
    const int h = std::min(roi_size_, frame.rows - y0);
    if (w <= 0 || h <= 0) {
      return;
    }

    cv::Mat roi = frame(cv::Rect(x0, y0, w, h)).clone();
    const int local_u = u - x0;
    const int local_v = v - y0;
    cv::rectangle(
      roi,
      cv::Rect(
        std::max(0, local_u - 2), std::max(0, local_v - 2),
        std::min(5, w), std::min(5, h)),
      cv::Scalar(0, 0, 255), 1);

    auto out = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, roi).toImageMsg();
    image_pub_->publish(*out);
  }

  std::unique_ptr<insta360_geometry::FisheyeCamera> camera_;
  int roi_size_{32};
  std::vector<Eigen::Quaterniond> markers_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuaternionVisualizer>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: Добавить исполняемый файл в `CMakeLists.txt`**

После блока `install(DIRECTORY include/ ...)`:

```cmake
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(quaternion_visualizer src/ros/quaternion_visualizer.cpp)
target_link_libraries(quaternion_visualizer
  insta360_geometry_core
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${cv_bridge_TARGETS}
)
install(TARGETS quaternion_visualizer
  DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 3: Собрать**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
```
Expected: сборка успешна, `quaternion_visualizer` установлен в `install/insta360_geometry/lib/insta360_geometry/`.

- [ ] **Step 4: Проверить, что узел запускается**

Run:
```bash
ros2 run insta360_geometry quaternion_visualizer --ros-args -p frame_width:=1920 &
sleep 2
ros2 node list | grep quaternion_visualizer
kill %1
```
Expected: узел появляется в списке.

- [ ] **Step 5: Commit**

```bash
git add insta360_geometry/src/ros/quaternion_visualizer.cpp \
  insta360_geometry/CMakeLists.txt
git commit -m "feat(geometry): quaternion_visualizer ROS node"
```

---

### Task 8: CLI `marker_cli`

**Files:**
- Create: `insta360_geometry/src/cli/marker_cli.cpp`
- Modify: `insta360_geometry/CMakeLists.txt` (добавить исполняемый файл)

**Interfaces:**
- Consumes: топик `~/marker_command` (Task 7).
- Produces: исполняемый файл `marker_cli`, читающий stdin и публикующий команды.

- [ ] **Step 1: Реализовать CLI**

`src/cli/marker_cli.cpp`:

```cpp
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("marker_cli");
  auto pub = node->create_publisher<std_msgs::msg::String>(
    "/quaternion_visualizer/marker_command", 10);

  std::cout << "Commands: draw w x y z | delete w x y z | clear | quit\n";
  std::string line;
  while (rclcpp::ok() && std::getline(std::cin, line)) {
    if (line == "quit" || line == "exit") {
      break;
    }
    if (line.empty()) {
      continue;
    }
    std_msgs::msg::String msg;
    msg.data = line;
    pub->publish(msg);
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: Добавить исполняемый файл в `CMakeLists.txt`**

```cmake
add_executable(marker_cli src/cli/marker_cli.cpp)
target_link_libraries(marker_cli
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
)
install(TARGETS marker_cli
  DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 3: Собрать**

Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
```
Expected: сборка успешна, `marker_cli` установлен.

- [ ] **Step 4: Проверить запуск**

Run:
```bash
echo "clear" | timeout 3 ros2 run insta360_geometry marker_cli
```
Expected: команда публикуется без ошибок.

- [ ] **Step 5: Commit**

```bash
git add insta360_geometry/src/cli/marker_cli.cpp \
  insta360_geometry/CMakeLists.txt
git commit -m "feat(geometry): marker_cli command-line publisher"
```

---

### Task 9: README пакета

**Files:**
- Create: `insta360_geometry/README.md`

**Interfaces:**
- Consumes: всё выше.
- Produces: документация по сборке, запуску и параметрам.

- [ ] **Step 1: Написать `README.md`**

Содержание: назначение пакета, зависимости, сборка (`colcon build --packages-select insta360_geometry`), запуск тестов, запуск узла с параметрами, формат команд CLI, ограничения (некалиброванные параметры, поворот ONE RS, разрешение).

- [ ] **Step 2: Commit**

```bash
git add insta360_geometry/README.md
git commit -m "docs(geometry): package README"
```

---

## Self-Review

**Spec coverage:**
- §4 архитектура (зонтик, пакет, ядро без rclcpp) → Task 1.
- §5 API ядра (`LensParams`, `FisheyeModel`, `EquidistantModel`, `FisheyeCamera`) → Tasks 1–6.
- §6 ROS-узел (потоковый, ROI 32×32, команды draw/delete/clear, CLI) → Tasks 7–8.
- §7 тесты (equidistant, camera, round-trip, известные значения) → Tasks 3–6.
- §8 сборка и запуск → Task 1 (сборка), Task 9 (README).
- §9 ограничения → Task 9 (README).
- §10 разделение слоёв → не требует кода в этой итерации.
- §11 будущие расширения → вне плана.

**Placeholder scan:** плейсхолдеров нет; все шаги содержат код или точные команды.

**Type consistency:** `LensParams{fx, fy, cx, cy}`, `FisheyeModel::pixelToRay/rayToPixel`, `FisheyeCamera::pixelToQuaternion/quaternionToPixel` — имена и сигнатуры совпадают во всех задачах. `EquidistantModel` наследует `FisheyeModel`. `frame_width` — `int` везде.

**Известное ограничение плана:** Task 6 содержит математику перевода оптическая↔body система, которую надо проверить round-trip тестом (Task 6, Step 1). Если round-trip не сойдётся, матрицы в Step 3 — первое место для проверки.
