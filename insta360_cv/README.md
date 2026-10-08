# insta360_cv

Детекция ArUco-маркеров на кадрах Insta360: для каждого маркера — центр,
размер, дистанция и кватернион направления луча в body-фрейме камеры.
Работает поверх абстракции `CameraModel` из `insta360_geometry`, поэтому
поддерживает два режима проекции: equirectangular (ERP) и dual-fisheye.

## Состав пакета

- **Ядро** `insta360_cv_core` — библиотека C++17 без `rclcpp`; зависимости
  Eigen, OpenCV и `insta360_geometry`.
  - `include/insta360_cv/detection.hpp` — `Detection { class_id, confidence,
    center, size_x, size_y, distance_m, orientation }`.
  - `include/insta360_cv/pose_estimator.hpp` — абстрактный интерфейс
    `estimateDistance(corners, marker_length_m)`.
  - `include/insta360_cv/marker_pose_estimator.hpp` — оценка дистанции для
    dual-fisheye: локальная пинхол-модель по параметрам объектива.
  - `include/insta360_cv/erp_pose_estimator.hpp` — оценка дистанции для ERP:
    виртуальная пинхол-камера, ориентированная вдоль центрального луча
    маркера, с фокусным расстоянием, масштабированным угловым разрешением
    ERP на широте маркера.
  - `include/insta360_cv/aruco_detector.hpp` — `ArucoDetector`, не зависит от
    проекции: принимает `CameraModel`, список `PoseEstimator` и селектор,
    отображающий центр маркера в индекс оценщика.
- **ROS-узел** `aruco_detector_node` — подписывается на изображение, публикует
  `~/detections` (`insta360_cv/msg/DetectionArray`) и `~/visualization`
  (`sensor_msgs/Image`) с отрисованными bbox и подписями.
- **Сообщения** — `msg/Detection.msg`, `msg/DetectionArray.msg`.

## Режимы работы

Режим задаётся параметром `mode` (значение по умолчанию `erp`). Режимы
работают раздельно: создаётся только бэкенд выбранного режима, неиспользуемый
не инстанцируется.

| `mode` | Топик подписки | Модель камеры | Оценщик дистанции |
|---|---|---|---|
| `erp` (по умолчанию) | `/equirectangular/image` | `EquirectangularCamera` | `ErpPoseEstimator` |
| `fisheye` | `/dual_fisheye/image` | `FisheyeCamera` | `MarkerPoseEstimator` (front/rear) |

Недопустимое значение `mode` приводит к исключению `std::invalid_argument`.

### Параметры режима `erp`

- `erp_width` (1920), `erp_height` (960) — размеры equirectangular-кадра.
- `erp_seam_margin_px` (0) — отступ от шва (x=0/x=W): детекции ближе к шву
  отбрасываются.
- `erp_pole_margin_px` (0) — отступ от полюсов (y=0/y=H): детекции ближе к
  полюсам отбрасываются.

### Параметры режима `fisheye`

- `frame_width` (1920) — ширина сырого dual-fisheye кадра.
- `front_fx/fy/cx/cy` (480), `rear_fx/fy/cx/cy` (480) — параметры объективов.

### Общие параметры

- `marker_length` (0.15) — сторона маркера в метрах.
- `dictionary` (9) — id словаря ArUco.
- `draw_overlay` (true) — публиковать ли `~/visualization`.

## Зависимости

`eigen`, `libopencv-dev`, `insta360_geometry`, `rclcpp`, `sensor_msgs`,
`std_msgs`, `geometry_msgs`, `cv_bridge`; тесты — `ament_cmake_gtest`. Всё
объявлено в `package.xml`.

## Сборка

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_cv
source install/setup.bash
```

## Тесты

```bash
colcon test --packages-select insta360_cv
colcon test-result --verbose
```

Тесты ядра (15): оценка дистанции для dual-fisheye (front/rear, смещение
кадра), оценка дистанции для ERP (осевой, внеосевой и наклонённый маркер,
маркер у полюса), вырожденные углы, фильтр полос отчуждения ERP (шов и
полюса), интеграционные тесты `ArucoDetector` (детекция в переднем и заднем
объективе, пустой кадр). Запускаются на ноутбуке, без железа.

## Запуск узла

ERP (по умолчанию):

```bash
ros2 run insta360_cv aruco_detector_node --ros-args \
  -p mode:=erp \
  -p erp_width:=1920 -p erp_height:=960 \
  -p erp_seam_margin_px:=16 -p erp_pole_margin_px:=16
```

Dual-fisheye:

```bash
ros2 run insta360_cv aruco_detector_node --ros-args \
  -p mode:=fisheye \
  -p frame_width:=1920 \
  -p front_fx:=480.0 -p front_fy:=480.0 -p front_cx:=480.0 -p front_cy:=480.0 \
  -p rear_fx:=480.0  -p rear_fy:=480.0  -p rear_cx:=480.0  -p rear_cy:=480.0
```

Просмотр результата:

```bash
rqt_image_view /aruco_detector/visualization
```

## Ограничения

- Параметры камеры не откалиброваны — значения по умолчанию взяты из
  геометрии кадра.
- Оценка дистанции в ERP использует локальную пинхол-аппроксимацию; точность
  падает вблизи полюсов, где угловое разрешение по долготе минимально
  (`cos(lat)→0`, пиксель покрывает максимальный угловой диапазон).
- Кадр BGR8 (не RGB): красный bbox — `(0, 0, 255)`.
