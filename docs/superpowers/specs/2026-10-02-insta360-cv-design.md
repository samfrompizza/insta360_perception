# insta360_cv — дизайн модуля детекции (aruco) → кватернион + дальность

Дата: 2026-10-02
Статус: утверждён к реализации

## 1. Цель

Модуль детекции объекта на сыром dual-fisheye кадре Insta360: найти объект,
отрисовать bounding box, вычислить кватернион, указывающий в центр этого
bounding box (в системе координат корпуса камеры, body frame), и — для первого
целевого сценария (судно с aruco-маркером) — оценить дальность до объекта.

Первая итерация (MVP) — **aruco-маркеры**: детекция `cv::aruco::detectMarkers`,
центр маркера как центр bounding box, кватернион направления через готовый
примитив `FisheyeCamera::pixelToQuaternion`, дальность через fisheye-→pinhole
undistort (`cv::fisheye::undistortPoints`) + pinhole `cv::solvePnP` по известному
физическому размеру маркера.

## 2. Контекст

### 2.1. Драйвер камеры

`insta360_ros_driver` (форк `ai4ce/insta360_ros_driver`, ветка `humble`, **не
трогаем**) публикует:

- `/dual_fisheye/image` — BGR8, 1920×960, сырой несошитый dual-fisheye кадр
  (две fisheye-окружности 960×960 side-by-side). Разрешение захардкожено
  (`RES_1920_960P30`, `src/main.cpp:130`).
- `/imu/data_raw` — SensorDataQoS (в этой итерации не используется).

Frame ID изображения — `camera_frame`. `camera_info`/интринсиков/модели
дисторсии драйвер **не публикует** — интринсики задаются ROS-параметрами узла.

### 2.2. Модуль геометрии (готов, переиспользуем)

`insta360_geometry` уже даёт нужный примитив — никакой новой геометрии писать
не надо:

```cpp
// insta360_geometry/fisheye_camera.hpp
class FisheyeCamera {
public:
  FisheyeCamera(LensParams front, LensParams rear, int frame_width,
                std::unique_ptr<const FisheyeModel> model);
  // пиксель (полный кадр 1920×960) -> body-frame кватернион направления
  std::optional<Eigen::Quaterniond> pixelToQuaternion(const cv::Point2d& pixel) const;
  std::optional<cv::Point2d> quaternionToPixel(const Eigen::Quaterniond& q) const;
};
```

- Семантика (A1): `q` переводит опорную ось `+X` body в направление луча.
- Body frame: X — вдоль оси объективов (front → +X, rear → −X), Y/Z — правая тройка.
- Выбор объектива автоматический: `pixel.x < frame_width/2` → front, иначе rear.
- `LensParams {double fx,fy,cx,cy}` — интринсики в **локальных** координатах
  линзы (центр 480,480 своей половины 960×960).

## 3. Решения (утверждены)

| Вопрос | Решение |
|---|---|
| Детектор (MVP) | aruco-маркеры (`cv::aruco`), на сыром fisheye-кадре без репроекции |
| Архитектура | core/ros split: `insta360_cv_core` (Eigen+OpenCV, без rclcpp) + тонкий ROS-узел |
| Система отсчёта | body frame (как в geometry); мировых координат CV не знает |
| Направление | `FisheyeCamera::pixelToQuaternion(центр bbox)` |
| Дальность | `cv::fisheye::undistortPoints` + pinhole `cv::solvePnP` по углам маркера и известному размеру; `distance = ‖tvec‖` |
| Оценка дальности живёт | внутри `insta360_cv` (согласно §10 спеки geometry) |
| Выход | bbox + центр + id + confidence + дальность + кватернион направления + визуализация |
| Сообщение | кастомное `Detection.msg` / `DetectionArray.msg` (ориентация встроена в `Detection`) |
| Конвертация Eigen→geometry_msgs | инлайн в ROS-узле (не в core) |
| Визуализация | свой оверлей в CV-узле; `quaternion_visualizer` не трогаем |
| Язык | C++17 |
| Обработка ошибок | `std::optional` / `NaN` для дальности; см. §8 |
| Тесты | gtest на ядро (без ROS-рантайма), как в geometry |
| GPU/NPU | не нужен для MVP (aruco на CPU); учтено для phase 2 (YOLO) |

## 4. Архитектура

Пакет `insta360_cv`, ament_cmake, C++17:

```
insta360_cv/
├── include/insta360_cv/
│   ├── detection.hpp          # struct Detection (id, bbox, confidence, distance, orientation)
│   └── aruco_detector.hpp     # core: детекция + pose/дальность + направление
├── src/
│   ├── aruco_detector.cpp     # вся математика, БЕЗ rclcpp
│   └── ros/aruco_detector_node.cpp  # тонкий ROS-узел
├── msg/
│   ├── Detection.msg
│   └── DetectionArray.msg
├── test/
│   ├── test_aruco_detector.cpp
│   └── test_pose_range.cpp
├── CMakeLists.txt
└── package.xml
```

**Правило зависимостей:** ядро (`detection.hpp`, `aruco_detector.hpp`) зависит
только от Eigen, OpenCV и `insta360_geometry_core` (переиспользует
`FisheyeCamera`). ROS-типы сообщений (`geometry_msgs`, `insta360_cv/msg`) — только
в слое ROS-узла. Направление: `ros/` → ядро → `insta360_geometry_core`.

### Компоненты

| Юнит | Назначение | Зависимости |
|---|---|---|
| `Detection` (core-структура) | результат одной детекции | Eigen (`Quaterniond`) |
| `ArucoDetector` (core) | `detectMarkers` → углы+id → центр bbox | OpenCV aruco |
| `MarkerPoseEstimator` (core) | fisheye undistort + pinhole solvePnP → `tvec` → дальность | OpenCV fisheye, `LensParams` |
| `ArucoDetectorNode` (ros) | подписка → core → публикация + оверлей | rclcpp, cv_bridge |

## 5. API ядра

### detection.hpp

```cpp
struct Detection {
  uint32_t class_id;           // для aruco — id маркера
  double   confidence;         // 0..1 (aruco: 1.0 при детекте)
  cv::Point2d center;          // центр bbox, пиксели полного кадра (1920×960)
  double   size_x, size_y;     // размеры bbox
  double   distance_m;         // дальность, метры; NaN если недоступна
  Eigen::Quaterniond orientation; // body-frame направление на центр
};
```

### aruco_detector.hpp

```cpp
class ArucoDetector {
public:
  ArucoDetector(int dictionary_id, double marker_length_m);

  // кадр (BGR) -> список детекций. Возвращает пустой вектор, если маркеров нет.
  std::vector<Detection> detect(const cv::Mat& frame_bgr) const;
};

class MarkerPoseEstimator {
public:
  MarkerPoseEstimator(LensParams lens);  // интринсики ЛОКАЛЬНОЙ линзы (480-центр)

  // углы маркера в ЛОКАЛЬНЫХ координатах линзы -> дальность (‖tvec‖).
  // nullopt при неудаче solvePnP.
  std::optional<double> estimateDistance(
      const std::vector<cv::Point2f>& corners_local, double marker_length_m) const;
};
```

`ArucoDetector::detect` внутри сам выбирает линзу (`center.x < frame_width/2` →
front, иначе rear), переводит углы в локальные координаты линзы, вызывает
`MarkerPoseEstimator` для дальности и `FisheyeCamera::pixelToQuaternion` для
направления. `FisheyeCamera` передаётся в `ArucoDetector` (или в узел), чтобы
ядро не инстанцировало геометрию дважды.

## 6. Сообщения

```msg
# insta360_cv/msg/Detection.msg
std_msgs/Header header
uint32  class_id
float64 confidence
float64 center_x
float64 center_y
float64 size_x
float64 size_y
float64 distance_m
geometry_msgs/Quaternion orientation   # body-frame направление на центр bbox
```

```msg
# insta360_cv/msg/DetectionArray.msg
std_msgs/Header header
insta360_cv/Detection[] detections
```

**Примечание (уточнение к варианту A из обсуждения):** вместо отдельного
`geometry_msgs/QuaternionStamped` ориентация встроена в `Detection`, чтобы
кватернион оставался привязан к своему маркеру при нескольких маркерах в кадре.
Frame ID и timestamp берутся из `header`.

## 7. ROS-узел `aruco_detector_node`

- **Подписка:** `/dual_fisheye/image` (`sensor_msgs/Image`, BGR8,
  `rclcpp::SensorDataQoS()` — как `quaternion_visualizer`).
- **Публикации:**
  - `~/detections` — `DetectionArray` (depth 10).
  - `~/visualization` — `sensor_msgs/Image` (оверлей: bbox + id + дальность).
- **Параметры** (`declare_parameter`, зеркалируя соглашение geometry):
  - `frame_width` (1920), `front_fx/fy/cx/cy` (480), `rear_fx/fy/cx/cy` (480).
  - `marker_length` (метры, физический размер стороны маркера — обязателен для
    дальности; значение по умолчанию задать в явном виде, например 0.15).
  - `dictionary` (id словаря aruco, по умолчанию `DICT_6X6_250`).
  - `draw_overlay` (bool, по умолчанию true).

**Оверлей:** `cv::rectangle` + `cv::putText` (id, `distance_m` с одним знаком
после запятой) на копии кадра → `cv_bridge::toImageMsg`. Кадр BGR8: красный =
`(0, 0, 255)`.

**Конвертация кватерниона (инлайн в узле):**

```cpp
geometry_msgs::msg::Quaternion toMsg(const Eigen::Quaterniond& q) {
  geometry_msgs::msg::Quaternion m;
  m.w = q.w(); m.x = q.x(); m.y = q.y(); m.z = q.z();
  return m;
}
```

## 8. Дальность на fisheye

- `cv::aruco::detectMarkers` дисторсия-агностична — работает на сыром fisheye.
- `cv::aruco::estimatePoseSingleMarkers` использует pinhole `cv::solvePnP` —
  **неприменим**. Равно как и `cv::fisheye::solvePnP`: в OpenCV 4.5.4 (идёт с
  ROS2 Humble) этой функции **нет** — она появилась только в OpenCV 4.9.
- Поэтому дальность оценивается двухшагово:
  1. `cv::fisheye::undistortPoints` переводит equidistant-пиксели маркера
     (r = f·θ) в pinhole-пиксели (r = f·tanθ);
  2. pinhole `cv::solvePnP` по этим точкам восстанавливает `tvec`.
  Это математически эквивалентно `cv::fisheye::solvePnP` из новых версий.
- Интринсики: `K = [f, 0, cx; 0, f, cy; 0, 0, 1]`. Коэффициенты дисторсии —
  **нули**: у fisheye `D` из 4 нулей (k=0 даёт `r = f·θ`, совпадает с нашей
  `EquidistantModel`), у pinhole `D` из 5 нулей.
- Углы маркера переводятся в локальные координаты линзы (центр 480,480):
  front `(x−480, y−480)`, rear `(x−960−480, y−480)`.
- Объектные точки — маркер в плоскости XY, углы `±marker_length/2` (конвенция aruco).
- `tvec` — центр маркера в оптической системе камеры; **дальность = `‖tvec‖`**.
- Публикуем body-frame кватернион (из `pixelToQuaternion`) + скаляр дальности —
  без смешивания систем координат. Направление и дальность согласованы (луч
  `pixelToRay` и `tvec` указывают на один и тот же маркер).

## 9. Обработка ошибок

- `pixelToQuaternion` вернул `nullopt` (центр вне fisheye-круга/FOV) → маркер
  пропускаем (не публикуем для него `Detection`).
- `detectMarkers` пусто → публикуем пустой `DetectionArray` + кадр без оверлея.
- `solvePnP` упал → `distance_m = NaN`, детекцию всё равно публикуем
  (направление надёжнее дальности).
- `cv_bridge` не смог конвертировать → лог + пропуск кадра.
- Невалидные параметры (`marker_length <= 0`, неизвестный `dictionary`) →
  исключение/логирование при старте узла.

## 10. Тестирование (TDD)

**Инфраструктура:** `ament_cmake_gtest`, блок `if(BUILD_TESTING)`, тесты в
`test/`. Запуск: `colcon test --packages-select insta360_cv`. Без железа, без
SDK, без ROS-рантайма.

1. **Детекция** (`test_aruco_detector.cpp`):
   - Синтетическое изображение с `cv::aruco::drawMarker` → корректный id и
     центр bbox.
   - Пустое изображение → пустой вектор.

2. **Дальность** (`test_pose_range.cpp`):
   - Маркер на известной дистанции, спроецированный `cv::fisheye::projectPoints`
     → `estimateDistance` восстанавливает дистанцию с допуском.
   - Неудачный pose (вырожденные углы) → `nullopt`.

3. **Направление:**
   - `pixelToQuaternion(центр)` сверяется с эталонным направлением (известная
     геометрия), кватернион нормирован.

**Не тестируем в первой итерации:** ROS-узел (тонкий, проверяется вручную),
как в geometry.

## 11. Сборка и запуск

**Зависимости (`package.xml`):** eigen, libopencv-dev, rclcpp, sensor_msgs,
std_msgs, geometry_msgs, image_transport, cv_bridge, `insta360_geometry`
(для ядра), rosidl_default_generators (для .msg); test: ament_cmake_gtest,
ament_lint_auto, ament_lint_common.

**Сборка:**
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_cv
source install/setup.bash
colcon test --packages-select insta360_cv
colcon test-result --verbose
```

**Запуск (два терминала):**
```bash
# T1: драйвер
ros2 launch insta360_ros_driver bringup.launch.xml

# T2: детектор
ros2 run insta360_cv aruco_detector_node --ros-args \
  -p marker_length:=0.15 -p dictionary:=DICT_6X6_250
```

**Просмотр:** `rqt_image_view /aruco_detector/visualization`,
`ros2 topic echo /aruco_detector/detections`.

## 12. Ограничения первой итерации

- Параметры камеры не откалиброваны — значения по умолчанию из геометрии кадра.
- Дальность точна только при корректно заданном `marker_length` и калибровке;
  при номинальных интринсиках (480) даёт оценку, а не точное измерение.
- `frame_width` должен совпадать с разрешением драйвера (1920).
- Один тип детектора (aruco); YOLO/репроекция — следующая итерация.
- Bringup-пакет отсутствует — запуск вручную.
- Кадр BGR8: красный = `(0, 0, 255)`.

## 13. Будущие расширения (вне первой итерации)

- YOLO-детекция людей/судов (phase 2) + репроекция fisheye→перспектива
  (tangent/cubemap) внутри ядра CV для CNN-детекторов.
- NPU-миграция на Orange Pi (RKNN/ncnn/ONNX-Runtime-NPU) — только для YOLO;
  aruco остаётся на CPU.
- Точная дальность после калибровки камеры; слияние с IMU/движением.
- Переход body → world (одометрия/VIO) — в `insta360_bringup`, не здесь.
