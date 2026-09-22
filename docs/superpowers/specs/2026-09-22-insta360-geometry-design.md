# insta360_geometry — дизайн модуля пиксель ↔ кватернион

Дата: 2026-09-22
Статус: утверждён к реализации

## 1. Цель

Модуль биективного отображения между пикселем на сыром dual-fisheye кадре
Insta360 и единичным кватернионом, кодирующим направление луча в системе
координат корпуса камеры (body frame). Модуль — основа для последующих задач
взаимного позиционирования (CV-детекция объекта → кватернион, стыковка с
панорамой, MAVLink).

## 2. Контекст

Существующий драйвер `insta360_ros_driver` (форк `ai4ce/insta360_ros_driver`,
ветка `humble`) публикует:

- `/dual_fisheye/image` — BGR8, 1920×960, сырой несошитый dual-fisheye кадр
  (две fisheye-окружности 960×960 side-by-side). Разрешение захардкожено в
  `src/main.cpp:130` (`RES_1920_960P30`).
- `/equirectangular/image` — rgb8, 1920×960, сошитая панорама (узел
  `equirectangular_cpp`, опционально).

В репозитории драйвера **нет** `camera_info`, интринсиков и модели дисторсии.
Узел equirectangular использует захардкоженную экдистантную модель
(`r = 2θ/π · W/2`) с руками подкрученными экстринсиками.

Драйвер не трогаем. Модуль геометрии — отдельный репозиторий-зонтик.

## 3. Решения (утверждены)

| Вопрос | Решение |
|---|---|
| Система отсчёта кватерниона | body frame камеры (X — вдоль оси объективов, front → +X, rear → −X; Y, Z перпендикулярны, правая тройка) |
| Семантика кватерниона | A1: поворот, переводящий опорную ось `+X` body в направление луча |
| Расширение A2 | чистая S²-кодировка (`[0, x, y, z]`) — позже, через открытый интерфейс, без ломания API |
| Модель проекции | экдистантная `r = f·θ`, за интерфейсом `FisheyeModel` (замена на Kannala–Brandt позже) |
| Параметры калибровки | отсутствуют; задаются конфигурируемо, значения по умолчанию из геометрии кадра |
| Язык | C++17 |
| Типы | `Eigen::Quaterniond` / `Eigen::Vector3d` для кватерниона/луча, `cv::Point2d` для пикселя |
| Выбор объектива | автоматически по `u < frame_width/2` → front, иначе rear |
| Обработка ошибок | `std::optional` при пикселе вне fisheye-круга / луче вне кадра; исключение только в конструкторе при невалидных параметрах |
| Владелец модели | `std::unique_ptr<const FisheyeModel>` |
| Разделение ядро/ROS | ядро — чистая C++ библиотека без `rclcpp`; ROS-узел — тонкая обёртка |
| Визуализация | потоковая, в реальном времени; ROI 32×32 вокруг каждого маркера; отдельная публикация на маркер |
| Команды визуализатору | ROS-топик команд + CLI-обёртка (`marker_cli`) |
| Тесты | gtest на ядро, запуск на ноутбуке (не на Orange Pi) |
| Bringup-пакет | не в первой итерации; запуск вручную тремя терминалами |

## 4. Архитектура

Репозиторий-зонтик `insta360_perception` в `~/ros2_ws/src/`, рядом с
`insta360_ros_driver` (не трогаем).

```
~/ros2_ws/src/
├── insta360_ros_driver/          ← существующий форк, не трогаем
└── insta360_perception/          ← новый репозиторий (git)
    ├── insta360_geometry/        ← пакет: ядро пиксель↔кватернион (первая итерация)
    ├── insta360_cv/              ← позже: детекция объекта → кватернион
    ├── insta360_mavlink/         ← позже: MAVLink-мост
    └── insta360_bringup/         ← позже: launch, конфиги, интеграция
```

Первый пакет — `insta360_geometry`, ament_cmake, C++17:

```
insta360_geometry/
├── include/insta360_geometry/
│   ├── lens_params.hpp          ← параметры объектива
│   ├── fisheye_model.hpp        ← интерфейс модели (абстракция)
│   ├── equidistant_model.hpp    ← экдистантная реализация
│   └── fisheye_camera.hpp       ← публичный API
├── src/
│   ├── equidistant_model.cpp
│   └── fisheye_camera.cpp
├── src/ros/
│   └── quaternion_visualizer.cpp ← ROS-узел (тонкая обёртка)
├── src/cli/
│   └── marker_cli.cpp           ← CLI-обёртка над топиком команд
├── test/
│   ├── test_equidistant_model.cpp
│   ├── test_fisheye_camera.cpp
│   └── test_roundtrip.cpp
├── CMakeLists.txt
└── package.xml
```

**Правило зависимостей:** ядро (`lens_params`, `fisheye_model`,
`equidistant_model`, `fisheye_camera`) не зависит от `rclcpp`. Зависимости
ядра — только Eigen и OpenCV (для `cv::Point2d`). Направление зависимостей:
`ros/`, `cli/` → ядро. Никогда наоборот.

## 5. API ядра

### lens_params.hpp

```cpp
struct LensParams {
  double fx, fy;   // фокус в пикселях
  double cx, cy;   // оптический центр
};
```

### fisheye_model.hpp

```cpp
class FisheyeModel {
public:
  virtual ~FisheyeModel() = default;  // виртуальный: полиморфное удаление
  // пиксель -> единичный луч в системе объектива
  virtual std::optional<Eigen::Vector3d> pixelToRay(
      const cv::Point2d& pixel, const LensParams& p) const = 0;
  // единичный луч -> пиксель
  virtual std::optional<cv::Point2d> rayToPixel(
      const Eigen::Vector3d& ray, const LensParams& p) const = 0;
};
```

### equidistant_model.hpp

Реализация `r = f·θ`, где `θ = atan2(√(x²+y²), z)`.
Обратно: `θ = r/f`, луч `(sin θ·cos φ, sin θ·sin φ, cos θ)`, где `φ` — азимут
пикселя относительно оптического центра.

### fisheye_camera.hpp

```cpp
class FisheyeCamera {
public:
  FisheyeCamera(LensParams front, LensParams rear, int frame_width,
                std::unique_ptr<const FisheyeModel> model);

  // пиксель -> кватернион-поворот (A1), опорная ось +X body
  std::optional<Eigen::Quaterniond> pixelToQuaternion(const cv::Point2d& pixel) const;

  // кватернион -> пиксель
  std::optional<cv::Point2d> quaternionToPixel(const Eigen::Quaterniond& q) const;
};
```

**Геометрия A1:** кватернион `q` такой, что
`q ⊗ [0,1,0,0] ⊗ q* = [0, x, y, z]` — переводит `+X` body в направление луча.
Обратное — `q` из луча через `Eigen::Quaterniond::FromTwoVectors(+X, ray)`.

**Выбор объектива:** внутри `FisheyeCamera` по `u < frame_width/2` (front) /
иначе (rear).

**Обработка ошибок:** `std::optional` при пикселе вне fisheye-круга / луче,
не попадающем в кадр. Исключение — только в конструкторе при невалидных
параметрах (`fx <= 0`, `frame_width <= 0`).

## 6. ROS-узел визуализации

**Узел `quaternion_visualizer`** — тонкая обёртка, потоковый режим:

- **Подписка:** `/dual_fisheye/image` через `image_transport` (raw).
- **Топик команд:** `~/marker_command` (`std_msgs/String`).
- **Публикация:** `~/visualization` (`sensor_msgs/Image`) — ROI 32×32 вокруг
  каждого маркера, отдельная публикация на маркер.
- **Параметры:** `frame_width`, `front_fx/fy/cx/cy`, `rear_fx/fy/cx/cy`,
  `roi_size` (по умолчанию 32).

**Семантика команд:**
- `draw [w, x, y, z]` — добавить маркер по кватерниону. Узел:
  `quaternionToPixel(q)` → пиксель → красный квадрат 32×32.
- `delete [w, x, y, z]` — удалить маркер с этим кватернионом (сравнение с
  угловым допуском, например < 1e-6).
- `clear` — удалить все маркеры.

**Хранение маркеров:** `std::vector<Eigen::Quaterniond>` активных маркеров.
На каждом кадре рисуются все.

**Эффективность:**
- Нет публикации полного кадра — только ROI 32×32 (~3 КБ вместо 5.5 МБ).
- Отрисовка `cv::rectangle` на 32×32 — микросекунды.
- ROI-срез `cv::Mat` — view, не копия.
- Единственная неизбежная стоимость — подписка на полный кадр (5.5 МБ × 30 FPS),
  та же, что у любого потребителя `/dual_fisheye/image`.

**CLI `marker_cli`:** читает stdin, парсит команды, публикует в
`~/marker_command`. Даёт удобный терминальный синтаксис поверх стандартного
ROS-топика.

**Позже (не в первой итерации):** метаданные для привязки ROI к маркеру
(вариант C), потоковая визуализация с идентичностью маркеров.

## 7. Тестирование (TDD)

**Инфраструктура:** `ament_cmake_gtest`, блок `if(BUILD_TESTING)`, тесты в
`test/`. Запуск: `colcon test --packages-select insta360_geometry`. Без железа,
без SDK, без ROS-рантайма.

**1. `EquidistantModel`:**
- Центр кадра → луч вдоль оптической оси (`θ=0`).
- Пиксель на известном радиусе → известный угол `θ = r/f`.
- Round-trip: `rayToPixel(pixelToRay(p)) == p` для набора пикселей внутри круга.
- Пиксель вне круга → `nullopt`.
- Луч за пределами FOV → `nullopt`.

**2. `FisheyeCamera`:**
- Выбор объектива: `u < W/2` → front, иначе rear.
- `pixelToQuaternion` → `quaternionToPixel` round-trip для обоих объективов.
- Кватернион нормирован (`|q| == 1`).
- Опорная ось: `q ⊗ [0,1,0,0] ⊗ q*` даёт направление луча.
- Невалидные параметры в конструкторе → исключение.

**3. Round-trip (главный):**
- Сетка пикселей по всему кадру → кватернион → пиксель → сравнение с исходным
  (допуск ~1e-6).

**4. Известные значения (оптимальный набор, без фанатизма):**
- Несколько пикселей с вручную посчитанными ожидаемыми кватернионами
  (центр, пиксель на 90° от центра, пиксель на границе круга).

**TDD-цикл:** тест (красный) → минимальная реализация (зелёный) → рефакторинг.

**Не тестируем в первой итерации:** ROS-узел (тонкий, проверяется вручную).

## 8. Сборка и запуск

**Сборка (ноутбук):**
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
colcon test --packages-select insta360_geometry
colcon test-result --verbose
```

**Зависимости:** `eigen3`, `opencv` (уже в системе), `rclcpp`, `sensor_msgs`,
`image_transport`, `cv_bridge`, `std_msgs` (для узла), `ament_cmake`,
`ament_cmake_gtest`. Всё объявлено в `package.xml`; `rosdep install` подтянет
недостающее.

**Запуск (три терминала):**
```bash
# T1: драйвер
ros2 launch insta360_ros_driver bringup.launch.xml

# T2: визуализатор
ros2 run insta360_geometry quaternion_visualizer --ros-args \
  -p frame_width:=1920 \
  -p front_fx:=480.0 -p front_fy:=480.0 -p front_cx:=480.0 -p front_cy:=480.0 \
  -p rear_fx:=480.0  -p rear_fy:=480.0  -p rear_cx:=480.0  -p rear_cy:=480.0

# T3: CLI команд
ros2 run insta360_geometry marker_cli
# > draw 0.0 0.707 0.0 0.707
# > delete 0.0 0.707 0.0 0.707
# > clear
```

**Просмотр:** `rqt_image_view /quaternion_visualizer/visualization`

## 9. Ограничения первой итерации

- Параметры камеры не откалиброваны — значения по умолчанию из геометрии кадра.
- Поворот ONE RS (сырой кадр повёрнут на 90°) — **не входит** в этот проект;
  фиксится отдельно в `decoder.cpp` драйвера. Геометрия исходит из корректной
  ориентации кадра.
- `frame_width` должен совпадать с разрешением в `main.cpp:130` (1920).
- Тесты гоняются на ноутбуке, не на Orange Pi.
- Bringup-пакет отсутствует — запуск вручную тремя терминалами.
- Кадр BGR8 (не RGB): красный = `(0, 0, 255)`.

## 10. Принцип разделения слоёв (для будущих модулей)

- **`insta360_geometry` и `insta360_cv` работают в body frame** и не знают про
  мировые координаты. Геометрия даёт кватернион луча; CV даёт объект +
  кватернион на него.
- **Оценка дальности до объекта живёт в `insta360_cv`**, а не в отдельном
  пакете и не в геометрии. Она может использовать оптические данные, IMU и
  движение камеры — это внутренняя деталь `insta360_cv`.
- **Переход body → world — отдельный слой** (одометрия/VIO), не часть геометрии
  и не часть CV. Композиция `q_world = q_odom ⊗ q_body` выполняется в
  `insta360_bringup` или отдельном узле-композиторе.
- Если для оценки дальности понадобится тяжёлая VIO/SfM-математика, она
  выносится в отдельный пакет, который `insta360_cv` использует.

## 11. Будущие расширения (вне первой итерации)

- `insta360_cv` — детекция объекта на fisheye-кадре → кватернион (body frame)
  + опциональная оценка дальности до объекта.
- Адаптер equirectangular-панорамы (`/equirectangular/image`) в модуль геометрии.
- Слой одометрии/VIO (body → world) — отдельный пакет или обёртка над готовой
  библиотекой.
- `insta360_mavlink` — MAVLink-мост для передачи данных на полётный контроллер.
- A2 (S²-кодировка) как дополнительный метод.
- Модель Kannala–Brandt вместо экдистантной.
- Метаданные привязки ROI к маркеру (вариант C).
- `insta360_bringup` — launch-файлы и конфиги.
