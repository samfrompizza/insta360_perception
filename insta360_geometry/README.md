# insta360_geometry

Биективное отображение между пикселем на сыром dual-fisheye кадре Insta360 и
единичным кватернионом, кодирующим направление луча в системе координат корпуса
камеры (body frame). Основа для последующих задач взаимного позиционирования:
CV-детекция объекта → кватернион, стыковка с панорамой, MAVLink.

## Состав пакета

- **Ядро** `insta360_geometry_core` — чистая библиотека C++17 без `rclcpp`;
  зависимости только Eigen и OpenCV.
  - `include/insta360_geometry/lens_params.hpp` — `LensParams { double fx, fy, cx, cy; }`.
    Параметры задаются в локальных координатах под-изображения объектива (оба
    объектива центрированы в локальной точке (480, 480) своего кадра 960×960).
  - `include/insta360_geometry/fisheye_model.hpp` — абстрактный интерфейс модели
    `pixelToRay` / `rayToPixel`.
  - `include/insta360_geometry/equidistant_model.hpp` — экдистантная реализация
    `r = f·θ`.
  - `include/insta360_geometry/fisheye_camera.hpp` — публичный API:
    `FisheyeCamera(LensParams front, LensParams rear, int frame_width,
    std::unique_ptr<const FisheyeModel> model)`; методы
    `pixelToQuaternion(const cv::Point2d&)` и
    `quaternionToPixel(const Eigen::Quaterniond&)` возвращают `std::optional`.
    Выбор объектива: `u < frame_width/2` → front, иначе rear. Кватернион —
    поворот, переводящий опорную ось `+X` body в направление луча. Смещение
    global-frame (сдвиг rear-объектива на `frame_width/2`) применяется внутри
    `FisheyeCamera`.
- **ROS-узел** `quaternion_visualizer` — тонкая потоковая обёртка. Подписывается
  на `/dual_fisheye/image` и `~/marker_command` (`std_msgs/String`), публикует
  `~/visualization` (`sensor_msgs/Image`) — ROI 32×32 вокруг каждого маркера.
- **CLI** `marker_cli` — читает команды со stdin и публикует их в топик
  `/quaternion_visualizer/marker_command`.

## Зависимости

`eigen`, `libopencv-dev`, `rclcpp`, `sensor_msgs`, `std_msgs`,
`image_transport`, `cv_bridge`; тесты — `ament_cmake_gtest`. Всё объявлено в
`package.xml`.

## Сборка

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
```

## Тесты

```bash
colcon test --packages-select insta360_geometry
colcon test-result --verbose
```

Тесты ядра (21): экдистантная модель, выбор объектива, round-trip
пиксель ↔ кватернион, известные значения. Запускаются на ноутбуке, без железа.

## Запуск узла

```bash
ros2 run insta360_geometry quaternion_visualizer --ros-args \
  -p frame_width:=1920 \
  -p front_fx:=480.0 -p front_fy:=480.0 -p front_cx:=480.0 -p front_cy:=480.0 \
  -p rear_fx:=480.0  -p rear_fy:=480.0  -p rear_cx:=480.0  -p rear_cy:=480.0 \
  -p roi_size:=32
```

Параметры (значения по умолчанию): `frame_width` (1920), `roi_size` (32),
`front_fx/fy/cx/cy` (480), `rear_fx/fy` (480), `rear_cx` (480), `rear_cy` (480).

Просмотр результата:

```bash
rqt_image_view /quaternion_visualizer/visualization
```

## Формат команд CLI

```bash
ros2 run insta360_geometry marker_cli
```

Команды (одна на строку):

| Команда | Описание |
|---|---|
| `draw w x y z` | добавить маркер по кватерниону `(w, x, y, z)` |
| `delete w x y z` | удалить маркер с этим кватернионом (сравнение с допуском) |
| `clear` | удалить все маркеры |
| `quit` | выйти |

Пример:

```
draw 0.0 0.707 0.0 0.707
delete 0.0 0.707 0.0 0.707
clear
quit
```

## Ограничения первой итерации

- Параметры камеры не откалиброваны — значения по умолчанию взяты из геометрии
  кадра.
- Поворот ONE RS (сырой кадр повёрнут на 90°) не входит в этот пакет; он
  фиксится отдельно в `decoder.cpp` драйвера. Геометрия исходит из корректной
  ориентации кадра.
- `frame_width` должен совпадать с разрешением, захардкоженным в драйвере (1920).
- Bringup-пакета нет — запуск вручную (драйвер, узел, CLI в трёх терминалах).
- Кадр BGR8 (не RGB): красный маркер — `(0, 0, 255)`.
