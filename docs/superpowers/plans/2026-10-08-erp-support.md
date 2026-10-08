# ERP Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Добавить в `insta360_perception` режим работы с ERP-кадром (equirectangular) для отображения «пиксель ↔ кватернион» и детекции ArUco-меток с оценкой расстояния, сохранив fisheye-режим как отдельно отключаемый бэкенд.

**Architecture:** Вводится интерфейс `CameraModel` с двумя реализациями — существующей `FisheyeCamera` и новой `EquirectangularCamera`. `ArucoDetector` становится модельно-независимым через интерфейс `PoseEstimator` (`MarkerPoseEstimator` для fisheye, `ErpPoseEstimator` для ERP). Узлы ROS выбирают бэкенд параметром `mode` при старте и конструируют только выбранный.

**Tech Stack:** C++17, ROS2 Humble, ament_cmake, Eigen3, OpenCV 4.5.4, gtest.

**Spec:** `docs/superpowers/specs/2026-10-08-erp-support-design.md`

## Global Constraints

- C++17, `-Wall -Wextra -Wpedantic` (уже включены в обоих CMakeLists).
- OpenCV 4.5.4 (Humble): нет `cv::fisheye::solvePnP`; для fisheye используется `cv::fisheye::undistortPoints` + pinhole `solvePnP`.
- Семантика кватерниона A1: кватернион поворачивает `+X` тела на направление луча. `+X` тела = ось передней линзы.
- Конвенция ERP (ground truth из драйвера): `lon = (x/W)·2π − π`, `lat = (y/H)·π − π/2`, `ray_erp = (cos(lat)sin(lon), sin(lat), cos(lat)cos(lon))`; `+Z_erp` вперёд, `+X_erp` вправо, `+Y_erp` вверх.
- Перевод ERP→тело: `ray_body = (ray_erp.z, −ray_erp.x, −ray_erp.y)` (совпадает с `opticalToBodyFront`).
- Новых пакетов и зависимостей не вводить; новые исходники — в существующие таргеты.
- Комментарии — только WHY, без emoji, без AI-filler.
- Каждый таск заканчивается коммитом.

---

### Task 1: Интерфейс `CameraModel` + `FisheyeCamera` как реализация

**Files:**
- Create: `insta360_geometry/include/insta360_geometry/camera_model.hpp`
- Modify: `insta360_geometry/include/insta360_geometry/fisheye_camera.hpp`
- Modify: `insta360_geometry/src/fisheye_camera.cpp`
- Test: `insta360_geometry/test/test_fisheye_camera.cpp` (существующий — регрессия)

**Interfaces:**
- Consumes: ничего.
- Produces: `insta360_geometry::CameraModel` — абстрактный класс с чисто виртуальными `pixelToQuaternion(const cv::Point2d&) const -> std::optional<Eigen::Quaterniond>` и `quaternionToPixel(const Eigen::Quaterniond&) const -> std::optional<cv::Point2d>`. `FisheyeCamera` наследует `CameraModel`.

- [ ] **Step 1: Создать `camera_model.hpp`**

```cpp
#ifndef INSTA360_GEOMETRY__CAMERA_MODEL_HPP_
#define INSTA360_GEOMETRY__CAMERA_MODEL_HPP_

#include <optional>

#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>

namespace insta360_geometry
{

/// Projection-agnostic camera: maps image pixels to unit quaternions in the
/// camera body frame, and back. Body frame: +X along the front lens axis.
/// Quaternion semantics A1: rotates body +X onto the ray direction.
class CameraModel
{
public:
  virtual ~CameraModel() = default;

  virtual std::optional<Eigen::Quaterniond> pixelToQuaternion(
    const cv::Point2d & pixel) const = 0;

  virtual std::optional<cv::Point2d> quaternionToPixel(
    const Eigen::Quaterniond & q) const = 0;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__CAMERA_MODEL_HPP_
```

- [ ] **Step 2: Сделать `FisheyeCamera` наследником `CameraModel`**

В `fisheye_camera.hpp` заменить include `fisheye_model.hpp` на `camera_model.hpp` (модель всё ещё нужна — оставить оба include), и объявить наследование + `override`:

```cpp
#include "insta360_geometry/camera_model.hpp"
#include "insta360_geometry/fisheye_model.hpp"
#include "insta360_geometry/lens_params.hpp"

namespace insta360_geometry
{

class FisheyeCamera : public CameraModel
{
public:
  FisheyeCamera(
    LensParams front, LensParams rear, int frame_width,
    std::unique_ptr<const FisheyeModel> model);

  std::optional<Eigen::Quaterniond> pixelToQuaternion(
    const cv::Point2d & pixel) const override;

  std::optional<cv::Point2d> quaternionToPixel(
    const Eigen::Quaterniond & q) const override;
  // ... private-часть без изменений
```

- [ ] **Step 3: Добавить `override` в `fisheye_camera.cpp`**

В определениях методов добавить `override` (сигнатуры не меняются):

```cpp
std::optional<Eigen::Quaterniond> FisheyeCamera::pixelToQuaternion(
  const cv::Point2d & pixel) const
```
→ оставить как есть (определение вне класса `override` не пишется). Достаточно, что заголовок объявляет `override`. Логику не трогать.

- [ ] **Step 4: Собрать и прогнать существующие тесты (регрессия)**

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_geometry && colcon test --packages-select insta360_geometry --event-handlers console_direct+
```
Expected: сборка без ошибок, все существующие тесты `test_fisheye_camera` PASS.

- [ ] **Step 5: Commit**

```bash
git add insta360_geometry/include/insta360_geometry/camera_model.hpp \
        insta360_geometry/include/insta360_geometry/fisheye_camera.hpp
git commit -m "refactor(geometry): extract CameraModel interface"
```

---

### Task 2: `EquirectangularCamera` — пиксель ↔ кватернион

**Files:**
- Create: `insta360_geometry/include/insta360_geometry/equirectangular_camera.hpp`
- Create: `insta360_geometry/src/equirectangular_camera.cpp`
- Modify: `insta360_geometry/CMakeLists.txt:18-21` (добавить исходник в `insta360_geometry_core`)
- Test: `insta360_geometry/test/test_equirectangular_camera.cpp`
- Modify: `insta360_geometry/CMakeLists.txt:70-80` (добавить gtest-таргет)

**Interfaces:**
- Consumes: `CameraModel` (Task 1).
- Produces: `insta360_geometry::EquirectangularCamera(int width, int height)` с `pixelToQuaternion` / `quaternionToPixel` (override). Возвращает `nullopt` для пикселей вне `[0,width)×[0,height)`.

- [ ] **Step 1: Написать падающий тест**

Создать `insta360_geometry/test/test_equirectangular_camera.cpp`:

```cpp
#include <gtest/gtest.h>

#include <cmath>

#include "insta360_geometry/equirectangular_camera.hpp"

using insta360_geometry::EquirectangularCamera;

namespace
{
constexpr int kW = 1920;
constexpr int kH = 960;
}  // namespace

TEST(EquirectangularCamera, RejectsNonPositiveDimensions)
{
  EXPECT_THROW(EquirectangularCamera(0, kH), std::invalid_argument);
  EXPECT_THROW(EquirectangularCamera(kW, 0), std::invalid_argument);
}

TEST(EquirectangularCamera, CenterPixelGivesIdentityRotation)
{
  EquirectangularCamera cam(kW, kH);
  // Center (W/2, H/2) -> lon=0, lat=0 -> ray_erp=(0,0,1) -> ray_body=(1,0,0).
  auto q = cam.pixelToQuaternion(cv::Point2d(kW / 2.0, kH / 2.0));
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(std::abs(q->w()), 1.0, 1e-9);
  EXPECT_NEAR(q->x(), 0.0, 1e-9);
  EXPECT_NEAR(q->y(), 0.0, 1e-9);
  EXPECT_NEAR(q->z(), 0.0, 1e-9);
}

TEST(EquirectangularCamera, QuaternionIsNormalized)
{
  EquirectangularCamera cam(kW, kH);
  auto q = cam.pixelToQuaternion(cv::Point2d(600.0, 400.0));
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(q->norm(), 1.0, 1e-12);
}

TEST(EquirectangularCamera, RoundTripGrid)
{
  EquirectangularCamera cam(kW, kH);
  for (double u = 1.0; u < kW; u += 120.0) {
    for (double v = 1.0; v < kH; v += 120.0) {
      auto q = cam.pixelToQuaternion(cv::Point2d(u, v));
      ASSERT_TRUE(q.has_value()) << "u=" << u << " v=" << v;
      auto px = cam.quaternionToPixel(*q);
      ASSERT_TRUE(px.has_value()) << "u=" << u << " v=" << v;
      EXPECT_NEAR(px->x, u, 1e-6) << "u=" << u << " v=" << v;
      EXPECT_NEAR(px->y, v, 1e-6) << "u=" << u << " v=" << v;
    }
  }
}

TEST(EquirectangularCamera, OutOfBoundsPixelReturnsNullopt)
{
  EquirectangularCamera cam(kW, kH);
  EXPECT_FALSE(cam.pixelToQuaternion(cv::Point2d(-1.0, 100.0)).has_value());
  EXPECT_FALSE(cam.pixelToQuaternion(cv::Point2d(kW, 100.0)).has_value());
  EXPECT_FALSE(cam.pixelToQuaternion(cv::Point2d(100.0, kH)).has_value());
}
```

- [ ] **Step 2: Зарегистрировать тест в CMake и убедиться, что он падает**

В `insta360_geometry/CMakeLists.txt` внутри `if(BUILD_TESTING)` добавить:

```cmake
  ament_add_gtest(test_equirectangular_camera test/test_equirectangular_camera.cpp)
  target_link_libraries(test_equirectangular_camera insta360_geometry_core)
```

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_geometry
```
Expected: FAIL сборки — `equirectangular_camera.hpp` не найден.

- [ ] **Step 3: Создать заголовок**

`insta360_geometry/include/insta360_geometry/equirectangular_camera.hpp`:

```cpp
#ifndef INSTA360_GEOMETRY__EQUIRECTANGULAR_CAMERA_HPP_
#define INSTA360_GEOMETRY__EQUIRECTANGULAR_CAMERA_HPP_

#include <optional>

#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>

#include "insta360_geometry/camera_model.hpp"

namespace insta360_geometry
{

/// Equirectangular (ERP) camera: maps pixels of a 2:1 equirectangular frame to
/// unit quaternions in the camera body frame, and back.
///
/// Convention matches insta360_ros_driver's equirectangular node:
///   lon = (x / width) * 2*pi - pi,  lat = (y / height) * pi - pi/2
///   ray_erp = (cos(lat)sin(lon), sin(lat), cos(lat)cos(lon))
/// with +Z_erp forward (image center), +X_erp right, +Y_erp up.
/// Body frame: +X along the front lens axis (ray_erp +Z -> body +X).
class EquirectangularCamera : public CameraModel
{
public:
  EquirectangularCamera(int width, int height);

  std::optional<Eigen::Quaterniond> pixelToQuaternion(
    const cv::Point2d & pixel) const override;

  std::optional<cv::Point2d> quaternionToPixel(
    const Eigen::Quaterniond & q) const override;

private:
  int width_;
  int height_;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__EQUIRECTANGULAR_CAMERA_HPP_
```

- [ ] **Step 4: Создать реализацию**

`insta360_geometry/src/equirectangular_camera.cpp`:

```cpp
#include "insta360_geometry/equirectangular_camera.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace insta360_geometry
{

EquirectangularCamera::EquirectangularCamera(int width, int height)
: width_(width), height_(height)
{
  if (width_ <= 0) {
    throw std::invalid_argument("EquirectangularCamera: width must be positive");
  }
  if (height_ <= 0) {
    throw std::invalid_argument("EquirectangularCamera: height must be positive");
  }
}

std::optional<Eigen::Quaterniond> EquirectangularCamera::pixelToQuaternion(
  const cv::Point2d & pixel) const
{
  if (pixel.x < 0.0 || pixel.x >= width_ || pixel.y < 0.0 || pixel.y >= height_) {
    return std::nullopt;
  }

  const double lon = (pixel.x / width_) * 2.0 * M_PI - M_PI;
  const double lat = (pixel.y / height_) * M_PI - M_PI / 2.0;

  const double cos_lat = std::cos(lat);
  const Eigen::Vector3d ray_erp(
    cos_lat * std::sin(lon), std::sin(lat), cos_lat * std::cos(lon));

  // ERP frame -> body frame: +Z_erp -> +X_body (front lens axis).
  const Eigen::Vector3d ray_body(ray_erp.z(), -ray_erp.x(), -ray_erp.y());

  return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), ray_body);
}

std::optional<cv::Point2d> EquirectangularCamera::quaternionToPixel(
  const Eigen::Quaterniond & q) const
{
  const Eigen::Vector3d ray_body = q * Eigen::Vector3d::UnitX();

  // Inverse of the ERP -> body mapping.
  const Eigen::Vector3d ray_erp(-ray_body.y(), -ray_body.z(), ray_body.x());

  const double lon = std::atan2(ray_erp.x(), ray_erp.z());
  const double lat = std::asin(std::clamp(ray_erp.y(), -1.0, 1.0));

  const double x = (lon + M_PI) / (2.0 * M_PI) * width_;
  const double y = (lat + M_PI / 2.0) / M_PI * height_;

  if (x < 0.0 || x >= width_ || y < 0.0 || y >= height_) {
    return std::nullopt;
  }
  return cv::Point2d(x, y);
}

}  // namespace insta360_geometry
```

- [ ] **Step 5: Добавить исходник в core-библиотеку**

В `insta360_geometry/CMakeLists.txt` в `add_library(insta360_geometry_core ...)` добавить строку:

```cmake
  src/equirectangular_camera.cpp
```

- [ ] **Step 6: Собрать и прогнать тесты**

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_geometry && colcon test --packages-select insta360_geometry --event-handlers console_direct+
```
Expected: PASS `test_equirectangular_camera` (все 5 тестов) и существующие тесты.

- [ ] **Step 7: Commit**

```bash
git add insta360_geometry/include/insta360_geometry/equirectangular_camera.hpp \
        insta360_geometry/src/equirectangular_camera.cpp \
        insta360_geometry/test/test_equirectangular_camera.cpp \
        insta360_geometry/CMakeLists.txt
git commit -m "feat(geometry): add EquirectangularCamera (pixel<->quaternion)"
```

---

### Task 3: Golden cross-check конвенции ERP

**Files:**
- Create: `insta360_geometry/test/test_erp_convention.cpp`
- Modify: `insta360_geometry/CMakeLists.txt:70-80` (добавить gtest-таргет)

**Interfaces:**
- Consumes: `EquirectangularCamera` (Task 2).
- Produces: ничего (только тест).

- [ ] **Step 1: Написать тест с жёстко зашитыми значениями**

Значения вычислены по формулам драйвера независимо от реализации. Создать `insta360_geometry/test/test_erp_convention.cpp`:

```cpp
#include <gtest/gtest.h>

#include <cmath>

#include "insta360_geometry/equirectangular_camera.hpp"

using insta360_geometry::EquirectangularCamera;

namespace
{
constexpr int kW = 1920;
constexpr int kH = 960;

// Expected body-frame ray for a pixel, computed from the driver's convention:
//   lon = (x/W)*2pi - pi, lat = (y/H)*pi - pi/2
//   ray_erp = (cos(lat)sin(lon), sin(lat), cos(lat)cos(lon))
//   ray_body = (ray_erp.z, -ray_erp.x, -ray_erp.y)
Eigen::Vector3d expectedRay(double x, double y)
{
  const double lon = (x / kW) * 2.0 * M_PI - M_PI;
  const double lat = (y / kH) * M_PI - M_PI / 2.0;
  const double cl = std::cos(lat);
  const Eigen::Vector3d erp(cl * std::sin(lon), std::sin(lat), cl * std::cos(lon));
  return Eigen::Vector3d(erp.z(), -erp.x(), -erp.y());
}
}  // namespace

TEST(ErpConvention, MatchesDriverFormulaAtKeyPixels)
{
  EquirectangularCamera cam(kW, kH);
  const double pixels[][2] = {
    {960.0, 480.0},   // center: lon=0, lat=0
    {0.0, 480.0},     // seam: lon=-pi
    {480.0, 480.0},   // lon=-pi/2
    {1440.0, 480.0},  // lon=+pi/2
    {960.0, 0.0},     // south pole: lat=-pi/2
    {960.0, 959.0},   // near north pole
    {300.0, 200.0},   // arbitrary
  };
  for (const auto & p : pixels) {
    auto q = cam.pixelToQuaternion(cv::Point2d(p[0], p[1]));
    ASSERT_TRUE(q.has_value()) << "x=" << p[0] << " y=" << p[1];
    const Eigen::Vector3d ray = q->toRotationMatrix() * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d exp = expectedRay(p[0], p[1]);
    EXPECT_NEAR(ray.x(), exp.x(), 1e-9) << "x=" << p[0] << " y=" << p[1];
    EXPECT_NEAR(ray.y(), exp.y(), 1e-9) << "x=" << p[0] << " y=" << p[1];
    EXPECT_NEAR(ray.z(), exp.z(), 1e-9) << "x=" << p[0] << " y=" << p[1];
  }
}
```

- [ ] **Step 2: Зарегистрировать тест в CMake**

В `insta360_geometry/CMakeLists.txt` внутри `if(BUILD_TESTING)` добавить:

```cmake
  ament_add_gtest(test_erp_convention test/test_erp_convention.cpp)
  target_link_libraries(test_erp_convention insta360_geometry_core)
```

- [ ] **Step 3: Собрать и прогнать**

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_geometry && colcon test --packages-select insta360_geometry --event-handlers console_direct+
```
Expected: PASS `test_erp_convention`.

- [ ] **Step 4: Commit**

```bash
git add insta360_geometry/test/test_erp_convention.cpp insta360_geometry/CMakeLists.txt
git commit -m "test(geometry): golden cross-check ERP convention against driver"
```

---

### Task 4: Интерфейс `PoseEstimator` + `MarkerPoseEstimator` как реализация

**Files:**
- Create: `insta360_cv/include/insta360_cv/pose_estimator.hpp`
- Modify: `insta360_cv/include/insta360_cv/marker_pose_estimator.hpp`
- Modify: `insta360_cv/src/marker_pose_estimator.cpp`
- Test: `insta360_cv/test/test_pose_range.cpp` (существующий — регрессия)

**Interfaces:**
- Consumes: ничего.
- Produces: `insta360_cv::PoseEstimator` — абстрактный класс с `estimateDistance(const std::vector<cv::Point2f>& corners, double marker_length_m) const -> std::optional<double>`. `MarkerPoseEstimator` наследует его.

- [ ] **Step 1: Создать `pose_estimator.hpp`**

```cpp
#ifndef INSTA360_CV__POSE_ESTIMATOR_HPP_
#define INSTA360_CV__POSE_ESTIMATOR_HPP_

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace insta360_cv
{

/// Estimates the distance to a square marker from its four projected corners.
class PoseEstimator
{
public:
  virtual ~PoseEstimator() = default;

  virtual std::optional<double> estimateDistance(
    const std::vector<cv::Point2f> & corners,
    double marker_length_m) const = 0;
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__POSE_ESTIMATOR_HPP_
```

- [ ] **Step 2: Сделать `MarkerPoseEstimator` наследником**

В `marker_pose_estimator.hpp` добавить include `pose_estimator.hpp` и наследование:

```cpp
#include "insta360_cv/pose_estimator.hpp"

namespace insta360_cv
{

class MarkerPoseEstimator : public PoseEstimator
{
public:
  explicit MarkerPoseEstimator(insta360_geometry::LensParams lens);

  std::optional<double> estimateDistance(
    const std::vector<cv::Point2f> & corners_local,
    double marker_length_m) const override;

private:
  insta360_geometry::LensParams lens_;
};
```

- [ ] **Step 3: Собрать и прогнать регрессию**

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_cv && colcon test --packages-select insta360_cv --event-handlers console_direct+
```
Expected: PASS `test_pose_range`, `test_aruco_detector`.

- [ ] **Step 4: Commit**

```bash
git add insta360_cv/include/insta360_cv/pose_estimator.hpp \
        insta360_cv/include/insta360_cv/marker_pose_estimator.hpp
git commit -m "refactor(cv): extract PoseEstimator interface"
```

---

### Task 5: `ErpPoseEstimator` — оценка расстояния на ERP

**Files:**
- Create: `insta360_cv/include/insta360_cv/erp_pose_estimator.hpp`
- Create: `insta360_cv/src/erp_pose_estimator.cpp`
- Modify: `insta360_cv/CMakeLists.txt:28-31` (добавить исходник в `insta360_cv_core`)
- Test: `insta360_cv/test/test_erp_pose.cpp`
- Modify: `insta360_cv/CMakeLists.txt:67-73` (добавить gtest-таргет)

**Interfaces:**
- Consumes: `PoseEstimator` (Task 4), `EquirectangularCamera` (Task 2).
- Produces: `insta360_cv::ErpPoseEstimator(const insta360_geometry::EquirectangularCamera& camera)` с `estimateDistance(const std::vector<cv::Point2f>& corners, double marker_length_m) const -> std::optional<double>`. `corners` — 4 угла в пикселях ERP-кадра.

- [ ] **Step 1: Написать падающий тест**

Создать `insta360_cv/test/test_erp_pose.cpp`. Тест проецирует синтетический маркер в ERP-пиксели через обратную к `ErpPoseEstimator` геометрию (лучи → ERP-пиксели), затем проверяет восстановление дистанции.

```cpp
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Geometry>

#include "insta360_cv/erp_pose_estimator.hpp"
#include "insta360_geometry/equirectangular_camera.hpp"

using insta360_cv::ErpPoseEstimator;
using insta360_geometry::EquirectangularCamera;

namespace
{
constexpr int kW = 1920;
constexpr int kH = 960;

// Project a marker of side `marker_length` at `distance_m` along body +X into
// ERP pixels. Marker plane is perpendicular to +X (facing the camera).
std::vector<cv::Point2f> projectMarker(double marker_length, double distance_m)
{
  const double m = marker_length / 2.0;
  // Marker corners in body frame: +X forward, +Y up, +Z right.
  const Eigen::Vector3d corners_body[4] = {
    {distance_m, m, -m}, {distance_m, m, m},
    {distance_m, -m, m}, {distance_m, -m, -m}};

  EquirectangularCamera cam(kW, kH);
  std::vector<cv::Point2f> out;
  for (const auto & c : corners_body) {
    const Eigen::Vector3d dir = c.normalized();
    const Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), dir);
    auto px = cam.quaternionToPixel(q);
    if (!px.has_value()) {
      return {};
    }
    out.push_back(cv::Point2f(static_cast<float>(px->x), static_cast<float>(px->y)));
  }
  return out;
}
}  // namespace

TEST(ErpPoseEstimator, RecoversDistanceFromProjectedMarker)
{
  const double marker_length = 0.15;
  const double true_distance = 2.0;
  const auto corners = projectMarker(marker_length, true_distance);
  ASSERT_EQ(corners.size(), 4u);

  EquirectangularCamera cam(kW, kH);
  ErpPoseEstimator estimator(cam);
  const auto distance = estimator.estimateDistance(corners, marker_length);

  ASSERT_TRUE(distance.has_value());
  EXPECT_NEAR(*distance, true_distance, true_distance * 0.10);
}

TEST(ErpPoseEstimator, ReturnsNulloptOnDegenerateCorners)
{
  const std::vector<cv::Point2f> degenerate{
    {960.0f, 480.0f}, {960.0f, 480.0f}, {960.0f, 480.0f}, {960.0f, 480.0f}};

  EquirectangularCamera cam(kW, kH);
  ErpPoseEstimator estimator(cam);
  EXPECT_FALSE(estimator.estimateDistance(degenerate, 0.15).has_value());
}
```

- [ ] **Step 2: Зарегистрировать тест и убедиться, что падает**

В `insta360_cv/CMakeLists.txt` внутри `if(BUILD_TESTING)` добавить:

```cmake
  ament_add_gtest(test_erp_pose test/test_erp_pose.cpp)
  target_link_libraries(test_erp_pose insta360_cv_core)
```

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_cv
```
Expected: FAIL — `erp_pose_estimator.hpp` не найден.

- [ ] **Step 3: Создать заголовок**

`insta360_cv/include/insta360_cv/erp_pose_estimator.hpp`:

```cpp
#ifndef INSTA360_CV__ERP_POSE_ESTIMATOR_HPP_
#define INSTA360_CV__ERP_POSE_ESTIMATOR_HPP_

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include <insta360_geometry/equirectangular_camera.hpp>

#include "insta360_cv/pose_estimator.hpp"

namespace insta360_cv
{

/// Distance estimation for markers detected in an equirectangular frame.
/// Uses a local pinhole approximation: a virtual pinhole camera is oriented
/// along the marker's central ray, with focal length scaled by the ERP angular
/// resolution at the marker's latitude.
class ErpPoseEstimator : public PoseEstimator
{
public:
  explicit ErpPoseEstimator(const insta360_geometry::EquirectangularCamera & camera);

  std::optional<double> estimateDistance(
    const std::vector<cv::Point2f> & corners,
    double marker_length_m) const override;

private:
  const insta360_geometry::EquirectangularCamera * camera_;
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__ERP_POSE_ESTIMATOR_HPP_
```

- [ ] **Step 4: Создать реализацию**

`insta360_cv/src/erp_pose_estimator.cpp`:

```cpp
#include "insta360_cv/erp_pose_estimator.hpp"

#include <cmath>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace insta360_cv
{

ErpPoseEstimator::ErpPoseEstimator(
  const insta360_geometry::EquirectangularCamera & camera)
: camera_(&camera)
{
}

std::optional<double> ErpPoseEstimator::estimateDistance(
  const std::vector<cv::Point2f> & corners,
  double marker_length_m) const
{
  if (corners.size() != 4 || marker_length_m <= 0.0) {
    return std::nullopt;
  }
  if (cv::contourArea(corners) <= 0.0) {
    return std::nullopt;
  }

  // Marker center direction (body frame) from the mean of the corner rays.
  Eigen::Vector3d center_dir = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> rays(4);
  for (std::size_t i = 0; i < 4; ++i) {
    auto q = camera_->pixelToQuaternion(cv::Point2d(corners[i].x, corners[i].y));
    if (!q.has_value()) {
      return std::nullopt;
    }
    rays[i] = q->toRotationMatrix() * Eigen::Vector3d::UnitX();
    center_dir += rays[i];
  }
  if (center_dir.norm() < 1e-9) {
    return std::nullopt;
  }
  center_dir.normalize();

  // Virtual pinhole camera looking along center_dir. Build a right-handed
  // basis (right, up, forward) and project each corner ray into it.
  const Eigen::Vector3d forward = center_dir;
  Eigen::Vector3d up_ref = Eigen::Vector3d::UnitY();
  if (std::abs(forward.dot(up_ref)) > 0.99) {
    up_ref = Eigen::Vector3d::UnitZ();
  }
  const Eigen::Vector3d right = forward.cross(up_ref).normalized();
  const Eigen::Vector3d up = right.cross(forward).normalized();

  // ERP angular resolution at the marker latitude: pixels per radian.
  const double lat = std::asin(std::clamp(forward.y(), -1.0, 1.0));
  const double f_local = (camera_->width() / (2.0 * M_PI)) * std::cos(lat);
  if (f_local <= 1e-6) {
    return std::nullopt;
  }

  const double cx = camera_->width() / 2.0;
  const double cy = camera_->height() / 2.0;
  const cv::Mat K = (cv::Mat_<double>(3, 3) <<
    f_local, 0.0, cx,
    0.0, f_local, cy,
    0.0, 0.0, 1.0);

  std::vector<cv::Point2f> pinhole(4);
  for (std::size_t i = 0; i < 4; ++i) {
    const double z = rays[i].dot(forward);
    if (z <= 1e-9) {
      return std::nullopt;
    }
    const double x = rays[i].dot(right) / z;
    const double y = rays[i].dot(up) / z;
    pinhole[i] = cv::Point2f(
      static_cast<float>(cx + f_local * x),
      static_cast<float>(cy - f_local * y));
  }

  const float m = static_cast<float>(marker_length_m / 2.0);
  const std::vector<cv::Point3f> object_points{
    {-m, m, 0.0f}, {m, m, 0.0f}, {m, -m, 0.0f}, {-m, -m, 0.0f}};

  const cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
  cv::Mat rvec;
  cv::Mat tvec;
  try {
    cv::solvePnP(object_points, pinhole, K, D, rvec, tvec);
  } catch (const cv::Exception &) {
    return std::nullopt;
  }

  return cv::norm(tvec);
}

}  // namespace insta360_cv
```

- [ ] **Step 5: Добавить аксессоры `width()`/`height()` в `EquirectangularCamera`**

`ErpPoseEstimator` использует `camera_->width()` и `camera_->height()`. В `equirectangular_camera.hpp` добавить публичные методы:

```cpp
  int width() const { return width_; }
  int height() const { return height_; }
```

- [ ] **Step 6: Добавить исходник в core-библиотеку**

В `insta360_cv/CMakeLists.txt` в `add_library(insta360_cv_core ...)` добавить:

```cmake
  src/erp_pose_estimator.cpp
```

- [ ] **Step 7: Собрать и прогнать тесты**

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_geometry insta360_cv && colcon test --packages-select insta360_cv --event-handlers console_direct+
```
Expected: PASS `test_erp_pose` (оба теста) и существующие тесты.

- [ ] **Step 8: Commit**

```bash
git add insta360_cv/include/insta360_cv/erp_pose_estimator.hpp \
        insta360_cv/src/erp_pose_estimator.cpp \
        insta360_cv/test/test_erp_pose.cpp \
        insta360_cv/CMakeLists.txt \
        insta360_geometry/include/insta360_geometry/equirectangular_camera.hpp
git commit -m "feat(cv): add ErpPoseEstimator (local pinhole distance)"
```

---

### Task 6: `ArucoDetector` — модельно-независимый

**Files:**
- Modify: `insta360_cv/include/insta360_cv/aruco_detector.hpp`
- Modify: `insta360_cv/src/aruco_detector.cpp`
- Test: `insta360_cv/test/test_aruco_detector.cpp` (существующий — регрессия)

**Interfaces:**
- Consumes: `CameraModel` (Task 1), `PoseEstimator` (Task 4).
- Produces: `ArucoDetector(int dictionary_id, double marker_length_m, const insta360_geometry::CameraModel& camera, const PoseEstimator& pose_estimator)` с `detect(const cv::Mat& frame_bgr) const -> std::vector<Detection>`. Детектор больше не делит кадр на половины и не знает про fisheye/ERP.

**Примечание:** fisheye-режим требует двух оценщиков (передний/задний) и деления кадра. Чтобы не потерять это, детектор принимает `std::vector<const PoseEstimator*>` и функцию выбора оценщика по пикселю. Для ERP — один оценщик на весь кадр; для fisheye — два, выбор по `center.x < frame_width/2`. Реализуем через `std::function<std::size_t(const cv::Point2d&)>` селектор.

- [ ] **Step 1: Обновить заголовок**

`insta360_cv/include/insta360_cv/aruco_detector.hpp`:

```cpp
#ifndef INSTA360_CV__ARUCO_DETECTOR_HPP_
#define INSTA360_CV__ARUCO_DETECTOR_HPP_

#include <functional>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <insta360_geometry/camera_model.hpp>

#include "insta360_cv/detection.hpp"
#include "insta360_cv/pose_estimator.hpp"

namespace insta360_cv
{

class ArucoDetector
{
public:
  /// `estimator_selector` maps a marker center pixel to an index into
  /// `estimators`. For ERP pass a single estimator and a selector returning 0;
  /// for dual-fisheye pass front/rear estimators and select by frame half.
  ArucoDetector(
    int dictionary_id,
    double marker_length_m,
    const insta360_geometry::CameraModel & camera,
    std::vector<const PoseEstimator *> estimators,
    std::function<std::size_t(const cv::Point2d &)> estimator_selector);

  std::vector<Detection> detect(const cv::Mat & frame_bgr) const;

private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  double marker_length_m_;
  const insta360_geometry::CameraModel * camera_;
  std::vector<const PoseEstimator *> estimators_;
  std::function<std::size_t(const cv::Point2d &)> estimator_selector_;
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__ARUCO_DETECTOR_HPP_
```

- [ ] **Step 2: Обновить реализацию**

`insta360_cv/src/aruco_detector.cpp`:

```cpp
#include "insta360_cv/aruco_detector.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace insta360_cv
{

ArucoDetector::ArucoDetector(
  int dictionary_id,
  double marker_length_m,
  const insta360_geometry::CameraModel & camera,
  std::vector<const PoseEstimator *> estimators,
  std::function<std::size_t(const cv::Point2d &)> estimator_selector)
: marker_length_m_(marker_length_m),
  camera_(&camera),
  estimators_(std::move(estimators)),
  estimator_selector_(std::move(estimator_selector))
{
  if (marker_length_m <= 0.0) {
    throw std::invalid_argument("marker_length_m must be > 0");
  }
  if (estimators_.empty()) {
    throw std::invalid_argument("at least one pose estimator is required");
  }
  if (!estimator_selector_) {
    throw std::invalid_argument("estimator_selector must be set");
  }
  try {
    dictionary_ = cv::aruco::getPredefinedDictionary(
      static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary_id));
  } catch (const cv::Exception &) {
    throw std::invalid_argument("unknown dictionary id");
  }
}

std::vector<Detection> ArucoDetector::detect(const cv::Mat & frame_bgr) const
{
  cv::Mat gray;
  cv::cvtColor(frame_bgr, gray, cv::COLOR_BGR2GRAY);

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids);

  std::vector<Detection> detections;
  detections.reserve(ids.size());

  for (std::size_t i = 0; i < ids.size(); ++i) {
    const cv::Rect bbox = cv::boundingRect(corners[i]);
    const cv::Point2d center(
      bbox.x + bbox.width / 2.0, bbox.y + bbox.height / 2.0);

    const auto orientation = camera_->pixelToQuaternion(center);
    if (!orientation) {
      continue;  // center outside the camera's valid region — skip
    }

    Detection d;
    d.class_id = static_cast<std::uint32_t>(ids[i]);
    d.confidence = 1.0;
    d.center = center;
    d.size_x = bbox.width;
    d.size_y = bbox.height;
    d.orientation = *orientation;

    const std::size_t idx = estimator_selector_(center);
    if (idx >= estimators_.size()) {
      continue;
    }
    const auto distance =
      estimators_[idx]->estimateDistance(corners[i], marker_length_m_);
    d.distance_m = distance ? *distance :
      std::numeric_limits<double>::quiet_NaN();

    detections.push_back(d);
  }

  return detections;
}

}  // namespace insta360_cv
```

- [ ] **Step 3: Обновить существующий тест `test_aruco_detector.cpp` под новый конструктор**

Прочитать текущий `insta360_cv/test/test_aruco_detector.cpp` и заменить конструирование `ArucoDetector` на новую сигнатуру: передать `FisheyeCamera` как `CameraModel&`, вектор из двух `MarkerPoseEstimator` (front/rear) и селектор `[](const cv::Point2d& c){ return c.x < 960.0 ? 0u : 1u; }`. Логика проверок не меняется.

- [ ] **Step 4: Собрать и прогнать тесты**

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_cv && colcon test --packages-select insta360_cv --event-handlers console_direct+
```
Expected: PASS `test_aruco_detector`, `test_pose_range`, `test_erp_pose`.

- [ ] **Step 5: Commit**

```bash
git add insta360_cv/include/insta360_cv/aruco_detector.hpp \
        insta360_cv/src/aruco_detector.cpp \
        insta360_cv/test/test_aruco_detector.cpp
git commit -m "refactor(cv): make ArucoDetector projection-agnostic"
```

---

### Task 7: `aruco_detector_node` — параметр `mode` и раздельные бэкенды

**Files:**
- Modify: `insta360_cv/src/ros/aruco_detector_node.cpp`

**Interfaces:**
- Consumes: `EquirectangularCamera` (Task 2), `ErpPoseEstimator` (Task 5), `ArucoDetector` (Task 6), `FisheyeCamera`/`MarkerPoseEstimator` (существующие).
- Produces: узел с параметром `mode` (`"erp"`|`"fisheye"`, default `"erp"`), `erp_width`, `erp_height`, `erp_seam_margin_px`, `erp_pole_margin_px`.

- [ ] **Step 1: Переписать конструктор узла**

В `aruco_detector_node.cpp` заменить тело конструктора и добавить поля. Ключевое: конструируется только выбранный бэкенд.

```cpp
  ArucoDetectorNode()
  : Node("aruco_detector")
  {
    const std::string mode = declare_parameter("mode", std::string("erp"));
    const double marker_length = declare_parameter("marker_length", 0.15);
    const int dictionary = declare_parameter("dictionary", 9);
    draw_overlay_ = declare_parameter("draw_overlay", true);

    if (mode == "erp") {
      const int erp_width = declare_parameter("erp_width", 1920);
      const int erp_height = declare_parameter("erp_height", 960);
      seam_margin_px_ = declare_parameter("erp_seam_margin_px", 0);
      pole_margin_px_ = declare_parameter("erp_pole_margin_px", 0);

      auto camera = std::make_unique<insta360_geometry::EquirectangularCamera>(
        erp_width, erp_height);
      auto estimator = std::make_unique<ErpPoseEstimator>(*camera);
      detector_ = std::make_unique<ArucoDetector>(
        dictionary, marker_length, *camera,
        std::vector<const PoseEstimator *>{estimator.get()},
        [](const cv::Point2d &) { return std::size_t{0}; });
      erp_camera_ = std::move(camera);
      erp_estimator_ = std::move(estimator);
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/equirectangular/image", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { onImage(msg); });
    } else if (mode == "fisheye") {
      const int frame_width = declare_parameter("frame_width", 1920);
      const insta360_geometry::LensParams front{
        declare_parameter("front_fx", 480.0),
        declare_parameter("front_fy", 480.0),
        declare_parameter("front_cx", 480.0),
        declare_parameter("front_cy", 480.0)};
      const insta360_geometry::LensParams rear{
        declare_parameter("rear_fx", 480.0),
        declare_parameter("rear_fy", 480.0),
        declare_parameter("rear_cx", 480.0),
        declare_parameter("rear_cy", 480.0)};

      auto camera = std::make_unique<insta360_geometry::FisheyeCamera>(
        front, rear, frame_width,
        std::make_unique<insta360_geometry::EquidistantModel>());
      auto front_est = std::make_unique<MarkerPoseEstimator>(front);
      auto rear_est = std::make_unique<MarkerPoseEstimator>(rear);
      const double half = frame_width / 2.0;
      detector_ = std::make_unique<ArucoDetector>(
        dictionary, marker_length, *camera,
        std::vector<const PoseEstimator *>{front_est.get(), rear_est.get()},
        [half](const cv::Point2d & c) { return c.x < half ? std::size_t{0} : std::size_t{1}; });
      fisheye_camera_ = std::move(camera);
      front_estimator_ = std::move(front_est);
      rear_estimator_ = std::move(rear_est);
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/dual_fisheye/image", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { onImage(msg); });
    } else {
      throw std::invalid_argument("mode must be 'erp' or 'fisheye'");
    }

    detections_pub_ = create_publisher<insta360_cv::msg::DetectionArray>(
      "~/detections", 10);
    visualization_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/visualization", 10);
  }
```

- [ ] **Step 2: Добавить фильтр шва/полюсов в `onImage`**

Перед публикацией отфильтровать детекции, попадающие в полосы отчуждения (только для ERP; для fisheye маржи равны 0 и фильтр не срабатывает). Вставить после `detector_->detect(...)`:

```cpp
    std::vector<Detection> detections = detector_->detect(cv_ptr->image);
    if (erp_camera_) {
      const double w = erp_camera_->width();
      const double h = erp_camera_->height();
      detections.erase(
        std::remove_if(detections.begin(), detections.end(),
          [&](const Detection & d) {
            const bool near_seam =
              d.center.x < seam_margin_px_ || d.center.x > w - seam_margin_px_;
            const bool near_pole =
              d.center.y < pole_margin_px_ || d.center.y > h - pole_margin_px_;
            return near_seam || near_pole;
          }),
        detections.end());
    }
```

Добавить `#include <algorithm>` и `#include <stdexcept>`.

- [ ] **Step 3: Обновить поля класса**

Заменить существующие поля на:

```cpp
  std::unique_ptr<insta360_geometry::EquirectangularCamera> erp_camera_;
  std::unique_ptr<ErpPoseEstimator> erp_estimator_;
  std::unique_ptr<insta360_geometry::FisheyeCamera> fisheye_camera_;
  std::unique_ptr<MarkerPoseEstimator> front_estimator_;
  std::unique_ptr<MarkerPoseEstimator> rear_estimator_;
  std::unique_ptr<ArucoDetector> detector_;
  bool draw_overlay_ = true;
  int seam_margin_px_ = 0;
  int pole_margin_px_ = 0;
```

Добавить include `insta360_cv/erp_pose_estimator.hpp`, `insta360_cv/marker_pose_estimator.hpp`, `insta360_geometry/equirectangular_camera.hpp`.

- [ ] **Step 4: Собрать**

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_cv
```
Expected: сборка без ошибок.

- [ ] **Step 5: Commit**

```bash
git add insta360_cv/src/ros/aruco_detector_node.cpp
git commit -m "feat(cv): add mode parameter (erp/fisheye) to aruco_detector_node"
```

---

### Task 8: `quaternion_visualizer` — параметр `mode`

**Files:**
- Modify: `insta360_geometry/src/ros/quaternion_visualizer.cpp`

**Interfaces:**
- Consumes: `EquirectangularCamera` (Task 2), `FisheyeCamera` (существующий).
- Produces: узел с параметром `mode` (`"erp"`|`"fisheye"`, default `"erp"`), подписка на `/equirectangular/image` или `/dual_fisheye/image`.

- [ ] **Step 1: Прочитать текущий файл**

Прочитать `insta360_geometry/src/ros/quaternion_visualizer.cpp` целиком, чтобы понять текущее конструирование `FisheyeCamera` и подписку.

- [ ] **Step 2: Ввести `mode` и раздельное конструирование**

Заменить прямое создание `FisheyeCamera` на выбор по параметру. Хранить камеру как `std::unique_ptr<insta360_geometry::CameraModel>`:

```cpp
    const std::string mode = declare_parameter("mode", std::string("erp"));
    if (mode == "erp") {
      const int erp_width = declare_parameter("erp_width", 1920);
      const int erp_height = declare_parameter("erp_height", 960);
      camera_ = std::make_unique<insta360_geometry::EquirectangularCamera>(
        erp_width, erp_height);
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/equirectangular/image", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { onImage(msg); });
    } else if (mode == "fisheye") {
      const int frame_width = declare_parameter("frame_width", 1920);
      const insta360_geometry::LensParams front{
        declare_parameter("front_fx", 480.0),
        declare_parameter("front_fy", 480.0),
        declare_parameter("front_cx", 480.0),
        declare_parameter("front_cy", 480.0)};
      const insta360_geometry::LensParams rear{
        declare_parameter("rear_fx", 480.0),
        declare_parameter("rear_fy", 480.0),
        declare_parameter("rear_cx", 480.0),
        declare_parameter("rear_cy", 480.0)};
      camera_ = std::make_unique<insta360_geometry::FisheyeCamera>(
        front, rear, frame_width,
        std::make_unique<insta360_geometry::EquidistantModel>());
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/dual_fisheye/image", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { onImage(msg); });
    } else {
      throw std::invalid_argument("mode must be 'erp' or 'fisheye'");
    }
```

Тип поля камеры сменить на `std::unique_ptr<insta360_geometry::CameraModel>`. Все вызовы `pixelToQuaternion`/`quaternionToPixel` работают через базовый интерфейс без изменений.

- [ ] **Step 3: Собрать**

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_geometry
```
Expected: сборка без ошибок.

- [ ] **Step 4: Commit**

```bash
git add insta360_geometry/src/ros/quaternion_visualizer.cpp
git commit -m "feat(geometry): add mode parameter (erp/fisheye) to quaternion_visualizer"
```

---

### Task 9: Документация

**Files:**
- Modify: `insta360_cv/README.md`
- Modify: `insta360_geometry/README.md`

**Interfaces:**
- Consumes: всё выше.
- Produces: ничего.

- [ ] **Step 1: Обновить `insta360_cv/README.md`**

Убрать утверждение «Пока не реализован». Добавить раздел про режимы: параметр `mode` (`erp`/`fisheye`), топики подписки (`/equirectangular/image` vs `/dual_fisheye/image`), параметры `erp_width`/`erp_height`/`erp_seam_margin_px`/`erp_pole_margin_px`, и что режимы работают раздельно (неиспользуемый бэкенд не создаётся).

- [ ] **Step 2: Обновить `insta360_geometry/README.md`**

Добавить раздел про `CameraModel` и `EquirectangularCamera`: конвенция ERP, семантика A1, параметр `mode` в `quaternion_visualizer`.

- [ ] **Step 3: Commit**

```bash
git add insta360_cv/README.md insta360_geometry/README.md
git commit -m "docs: document ERP mode and CameraModel abstraction"
```

---

### Task 10: Финальная проверка сборки и тестов

**Files:** нет изменений.

- [ ] **Step 1: Полная сборка и тесты обоих пакетов**

Run:
```bash
cd /home/gasem/ros2_ws && colcon build --packages-select insta360_geometry insta360_cv && colcon test --packages-select insta360_geometry insta360_cv --event-handlers console_direct+
```
Expected: сборка без ошибок и предупреждений; все тесты PASS:
`test_lens_params`, `test_fisheye_model`, `test_equidistant_model`, `test_fisheye_camera`, `test_equirectangular_camera`, `test_erp_convention`, `test_pose_range`, `test_aruco_detector`, `test_erp_pose`.

- [ ] **Step 2: Проверить, что fisheye-режим не регрессировал**

Run:
```bash
cd /home/gasem/ros2_ws && colcon test-result --verbose
```
Expected: 0 failures.

- [ ] **Step 3: Commit (если были правки)**

Если шаги 1–2 потребовали правок — закоммитить их отдельно с описанием. Иначе таск завершён без коммита.
